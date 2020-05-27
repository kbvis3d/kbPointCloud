#version 460
#pragma optionNV(fastmath off)
#pragma optionNV(fastprecision off)

layout(location = 0) in vec4 color;
layout(location = 1) flat in int fragVisible;

layout(location = 0) out vec4 fragColor;

void main()
{
	if (fragVisible > 0)
	{
		fragColor = color;
	}
	else
	{
		discard;
	}
}
