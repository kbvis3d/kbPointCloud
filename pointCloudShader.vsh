#version 460
#pragma optionNV(fastmath off)
#pragma optionNV(fastprecision off)

#define KERNEL_PASS_NONE 0
#define KERNEL_PASS_KMEANS_OUTLIER_PARTITION 1
#define KERNEL_PASS_KMEANS_OUTLIER_KNN 2
#define KERNEL_PASS_KMEANS_OUTLIER_MEAN 3
#define KERNEL_PASS_KMEANS_OUTLIER_FILTER 4

layout(std430, binding = 0) buffer selectionBuffer {
	uint selectionIDs[];
};
layout(std430, binding = 1) buffer filterBuffer {
	uint filterValues[];
};
layout(std430, binding = 2) buffer filteredPointsBuffer {
	float filteredPoints[];
};
layout(std430, binding = 3) buffer verticesBuffer {
	float vertices[];
};
layout(std430, binding = 4) buffer distancesBuffer {
	uint distances[];
};
layout(std430, binding = 5) buffer splatCountsBuffer {
	uint splatCounts[];
};

layout(location = 0) uniform mat4  ProjectionMatrix;
layout(location = 1) uniform mat4  ModelViewMatrix;
layout(location = 2) uniform float PointScale = 1.0;
layout(location = 3) uniform vec3 BoundingBoxMin;
layout(location = 4) uniform vec3 BoundingBoxSize;
layout(location = 5) uniform ivec2 FilterPass = ivec2(0, 0);
layout(location = 6) uniform ivec3 FilterResolution;
layout(location = 7) uniform vec3 FilterTolerance;
layout(location = 8) uniform ivec3 FilterMaxKN;

layout(location = 0) in vec3 pointCenterIn;
layout(location = 1) in float pointColorIn;
layout(location = 2) in vec3 pointRgbIn;
	
layout(location = 0) out vec4 color;
layout(location = 1) flat out int fragVisible;

ivec3 getCell(vec3 pointCoord)
{
	vec3 offsetInBox = (pointCoord - BoundingBoxMin) / BoundingBoxSize;
	return ivec3(
		int(round((FilterResolution.x - 1) * clamp(offsetInBox.x, 0.0, 1.0))),
		int(round((FilterResolution.y - 1) * clamp(offsetInBox.y, 0.0, 1.0))),
		int(round((FilterResolution.z - 1) * clamp(offsetInBox.z, 0.0, 1.0))));
}
uint getCellIndex(vec3 pointCoord)
{
	vec3 offsetInBox = (pointCoord - BoundingBoxMin) / BoundingBoxSize;
	uint i = uint(round((FilterResolution.x - 1) * clamp(offsetInBox.x, 0.0, 1.0)));
	uint j = uint(round((FilterResolution.y - 1) * clamp(offsetInBox.y, 0.0, 1.0)));
	uint k = uint(round((FilterResolution.z - 1) * clamp(offsetInBox.z, 0.0, 1.0)));
	return uint((k * FilterResolution.y + j) * FilterResolution.x + i);
}
int[27] getCellIndices27(vec3 pointCoord)
{
	vec3 offsetInBox = (pointCoord - BoundingBoxMin) / BoundingBoxSize;
	int i = int(round((FilterResolution.x - 1) * clamp(offsetInBox.x, 0.0, 1.0)));
	int j = int(round((FilterResolution.y - 1) * clamp(offsetInBox.y, 0.0, 1.0)));
	int k = int(round((FilterResolution.z - 1) * clamp(offsetInBox.z, 0.0, 1.0)));
	int strideY = FilterResolution.x;
	int strideZ = FilterResolution.x * FilterResolution.y;
	int index = k * strideZ + j * strideY + i;
	int minus = -(index + strideZ + strideY + 1);
	int left = i > 0 ? -1 : minus;
	int right = i < FilterResolution.x - 1 ? 1 : minus;
	int bottom = j > 0 ? -strideY : minus;
	int top = j < FilterResolution.y - 1 ? strideY : minus;
	int back = k > 0 ? -strideZ : minus;
	int front = k < FilterResolution.z - 1 ? strideZ : minus;
	return int[27](
		index + back + bottom + left,
		index + back + bottom,
		index + back + bottom + right,
		index + back + left,
		index + back,
		index + back + right,
		index + back + top + left,
		index + back + top,
		index + back + top + right,

		index + bottom + left,
		index + bottom,
		index + bottom + right,
		index + left,
		index,
		index + right,
		index + top + left,
		index + top,
		index + top + right,

		index + front + bottom + left,
		index + front + bottom,
		index + front + bottom + right,
		index + front + left,
		index + front,
		index + front + right,
		index + front + top + left,
		index + front + top,
		index + front + top + right
		);
}
int[8] getCellIndices8(vec3 pointCoord)
{
	vec3 offsetInBox = (pointCoord - BoundingBoxMin) / BoundingBoxSize;
	float p = (FilterResolution.x - 1) * clamp(offsetInBox.x, 0.0, 1.0);
	float q = (FilterResolution.y - 1) * clamp(offsetInBox.y, 0.0, 1.0);
	float r = (FilterResolution.z - 1) * clamp(offsetInBox.z, 0.0, 1.0);
	float dx = fract(p);
	float dy = fract(q);
	float dz = fract(r);
	int i = int(round(p));
	int j = int(round(q));
	int k = int(round(r));
	int strideX = 1;
	int strideY = FilterResolution.x;
	int strideZ = FilterResolution.x * FilterResolution.y;
	int index = k * strideZ + j * strideY + i;
	int minus = -(index + strideZ + strideY + strideX);
	strideX = strideY = strideZ = minus;
	float thresholdMin = 0.5;
	float thresholdMax = 1.0 - thresholdMin;
	if (dx > thresholdMax && i > 0)
	{
		strideX = -1;
	}
	else if (dx < thresholdMin && i < FilterResolution.x - 1)
	{
		strideX = 1;
	}
	if (dy > thresholdMax && j > 0)
	{
		strideY = -FilterResolution.x;
	}
	else if (dy < thresholdMin && j < FilterResolution.y - 1)
	{
		strideY = FilterResolution.x;
	}
	if (dz > thresholdMax && k > 0)
	{
		strideZ = -FilterResolution.x * FilterResolution.y;
	}
	else if (dz < thresholdMin && k < FilterResolution.z - 1)
	{
		strideZ = FilterResolution.x * FilterResolution.y;
	}

	return int[8](
		index,
		index + strideX,
		index + strideY,
		index + strideY + strideX,
		index + strideZ,
		index + strideZ + strideX,
		index + strideZ + strideY,
		index + strideZ + strideY + strideX
		);
}

void main()
{
	const float SUM_SCALE = (pow(2.0, 32.0) - 1.0) / float(FilterMaxKN.y);
	const float DISTANCE_SCALE = float(FilterPass.y - 1);
	gl_Position = vec4(0.0);
	gl_PointSize = 0.0;
	fragVisible = 1;

	if (FilterPass.x == KERNEL_PASS_KMEANS_OUTLIER_PARTITION)
	{
		uint cellIndex = getCellIndex(pointCenterIn);
		uint cellTableIndex = cellIndex * FilterMaxKN.x;
		uint k = atomicAdd(filterValues[cellTableIndex], 1);
		int vertexId = gl_VertexID + FilterMaxKN.z;
		if (k < FilterMaxKN.x - 1)
		{
			filterValues[cellTableIndex + k + 1] = vertexId;
		}
		int vertexIndex = vertexId * 3;
		vertices[vertexIndex] = pointCenterIn.x;
		vertices[vertexIndex + 1] = pointCenterIn.y;
		vertices[vertexIndex + 2] = pointCenterIn.z;
		fragVisible = 0;
	}
	else if (FilterPass.x == KERNEL_PASS_KMEANS_OUTLIER_KNN)
	{
		int vertexId = gl_VertexID + FilterMaxKN.z;
		uint distancesIndex = gl_VertexID * FilterPass.y;
		int[] cellIndices = getCellIndices8(pointCenterIn);
		for (int c = 0; c < 8; ++c)
		{
			int cellIndex = cellIndices[c];
			if (cellIndex >= 0)
			{
				uint cellTableIndex = cellIndex * FilterMaxKN.x;
				uint k = min(FilterMaxKN.x - 1, filterValues[cellTableIndex]);
				if (k > 0)
				{
					for (int i = 1; i <= k; ++i)
					{
						uint neighborId = filterValues[cellTableIndex + i];
						if (neighborId != vertexId)
						{
							uint vertexIndex = neighborId * 3;
							vec3 neighborPos = vec3(vertices[vertexIndex], vertices[vertexIndex + 1], vertices[vertexIndex + 2]);
							uint d = int(round(DISTANCE_SCALE * clamp(distance(pointCenterIn, neighborPos) / FilterTolerance.z, 0.0, 1.0)));
							atomicAdd(distances[distancesIndex + d], 1);
						}
					}
				}
			}
		}
		fragVisible = 0;
	}
	else if (FilterPass.x == KERNEL_PASS_KMEANS_OUTLIER_MEAN)
	{
		int vertexId = gl_VertexID + FilterMaxKN.z;
		int distancesIndex = gl_VertexID * FilterPass.y;
		uint K = uint(FilterTolerance.x);
		float sum = 0.0;
		uint samples = 0;
		bool finished = false;
		for (uint i = 0; i < FilterPass.y; ++i)
		{
			uint currentFrequency = distances[distancesIndex + i];
			if (samples + currentFrequency >= K)
			{
				currentFrequency = K - samples;
			}
			samples += currentFrequency;
			sum += currentFrequency * i;
			if (samples >= K)break;
		}
		float meanDistance = sum / (samples * DISTANCE_SCALE);
		uint meanDistanceUint = uint(round(meanDistance * SUM_SCALE));
		splatCounts[vertexId] = meanDistanceUint;
		atomicAdd(splatCounts[FilterMaxKN.y], meanDistanceUint);
		atomicAdd(splatCounts[FilterMaxKN.y + 1], uint(round(SUM_SCALE * meanDistance * meanDistance)));
		atomicAdd(splatCounts[FilterMaxKN.y + 2], 1);
		fragVisible = 0;
	}
	else if (FilterPass.x == KERNEL_PASS_KMEANS_OUTLIER_FILTER)
	{
		int vertexId = gl_VertexID + FilterMaxKN.z;
		float currentMean = float(splatCounts[vertexId]);
		float sigmaX = float(splatCounts[FilterMaxKN.y]);
		float N = float(splatCounts[FilterMaxKN.y + 2]);
		float mean = sigmaX / N;
		float sigmaXsq = float(splatCounts[FilterMaxKN.y + 1]);
		float stddev = sqrt(abs(sigmaXsq / N - mean * mean));
		if (currentMean > (mean + FilterTolerance.y * stddev))
		{
			fragVisible = 0;
		}
	}

	if (FilterPass.x == KERNEL_PASS_KMEANS_OUTLIER_FILTER || FilterPass.x == KERNEL_PASS_NONE)
	{
		color = fragVisible > 0 ? vec4(pointRgbIn, 1.0) : vec4(1.0, 0.0, 0.0, 1.0);
		fragVisible = 1;
	}

	if (fragVisible > 0)
	{
		vec4 posVS = ModelViewMatrix * vec4(pointCenterIn, 1.0);
		gl_Position = vec4(ProjectionMatrix * posVS);
		gl_PointSize = PointScale;
	}
}