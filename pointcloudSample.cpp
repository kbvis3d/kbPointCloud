/*
 * Copyright 2020 KBVIS Technologies.  All rights reserved.
 *
 * Please refer to the KBVIS end user license agreement (EULA) associated
 * with this source code for terms and conditions that govern your use of
 * this software. Any use, reproduction, disclosure, or distribution of
 * this software and related documentation outside the terms of the EULA
 * is strictly prohibited.
 *
 */

/*
    Point cloud rendering sample

    This sample loads a point cloud from LAS/LAZ and displays itront.
*/

#include "pointcloudtools/PointCloudTools.h"
#include<iostream>
#include <cassert>

using namespace std;
typedef unsigned  char byte;

// OpenGL Graphics includes
#include <helper_gl.h>
#include <GL/freeglut.h>
#include <GL/glew.h>

// CUDA Runtime, Interop, and includes
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include <cuda_profiler_api.h>
#include <vector_types.h>
#include <vector_functions.h>
#include <driver_functions.h>

// CUDA utilities
#include <helper_cuda.h>

// Helper functions
#include <helper_cuda.h>
#include <helper_functions.h>
#include <helper_timer.h>

typedef unsigned int uint;
typedef unsigned char uchar;


void loadLasFile();
void loadPointCloud();
void renderPointCloud();

const char *sSDKsample = "kbPunktWolke";

uint width = 512, height = 512;
dim3 blockSize(16, 16);
dim3 gridSize;

double scale = 1.0;
float3 viewRotation;
float3 viewTranslation = make_float3(0.0, 0.0, -4.0f);

StopWatchInterface *timer = 0;

// Auto-Verification Code
const int frameCheckNumber = 2;
int fpsCount = 0;        // FPS count for averaging
int fpsLimit = 1;        // FPS limit for sampling
int g_Index = 0;
unsigned int frameCount = 0;

int *pArgc;
char **pArgv;

#ifndef MAX
#define MAX(a,b) ((a > b) ? a : b)
#endif

void computeFPS()
{
    frameCount++;
    fpsCount++;

    if (fpsCount == fpsLimit){
        char fps[256];
        float ifps = 1.f / (sdkGetAverageTimerValue(&timer) / 1000.f);
        sprintf(fps, "Point Cloud Render: %3.1f fps", ifps);

        glutSetWindowTitle(fps);
        fpsCount = 0;

        fpsLimit = (int)MAX(1.f, ifps);
        sdkResetTimer(&timer);
    }
}

void render()
{
    //uint *d_output;
    //// map PBO to get CUDA device pointer
    //checkCudaErrors(cudaGraphicsMapResources(1, &cuda_pbo_resource, 0));
    //size_t num_bytes;
    //checkCudaErrors(cudaGraphicsResourceGetMappedPointer((void **)&d_output, &num_bytes, cuda_pbo_resource));

    //// clear image
    //checkCudaErrors(cudaMemset(d_output, 0, width*height*4));

    //// call CUDA kernel, writing results to PBO
    //render_kernel(gridSize, blockSize, d_output, width, height, density, brightness, transferOffset, transferScale);

    //getLastCudaError("kernel failed");

    //checkCudaErrors(cudaGraphicsUnmapResources(1, &cuda_pbo_resource, 0));
}

void idle()
{
    glutPostRedisplay();
}

int ox, oy;
int buttonState = 0;

void mouse(int button, int state, int x, int y)
{
    if (state == GLUT_DOWN) {
        buttonState |= 1 << button;
    }
    else if (state == GLUT_UP) {
        buttonState = 0;
    }

    ox = x;
    oy = y;
    glutPostRedisplay();
}

void motion(int x, int y)
{
    float dx, dy;
    dx = (float)(x - ox);
    dy = (float)(y - oy);

    if (buttonState == 4){
        // right = zoom
        viewTranslation.z += dy / 100.0f;
    }
    else if (buttonState == 2){
        // middle = translate
        viewTranslation.x += dx / 100.0f;
        viewTranslation.y -= dy / 100.0f;
    }
    else if (buttonState == 1){
        // left = rotate
        viewRotation.x += dy / 5.0f;
        viewRotation.y += dx / 5.0f;
    }

    ox = x;
    oy = y;
    glutPostRedisplay();
}

int iDivUp(int a, int b)
{
    return (a % b != 0) ? (a / b + 1) : (a / b);
}

void reshape(int w, int h)
{
    width = w;
    height = h;

    // calculate new grid size
    //gridSize = dim3(iDivUp(width, blockSize.x), iDivUp(height, blockSize.y));

    glViewport(0, 0, w, h);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);
}

void cleanup()
{
    sdkDeleteTimer(&timer);

    //freeCudaBuffers();
    //// Calling cudaProfilerStop causes all profile data to be
    //// flushed before the application exits
    //checkCudaErrors(cudaProfilerStop());
}

void initGL(int *argc, char **argv)
{
    glutInit(argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
    glutInitWindowSize(width, height);
    glutCreateWindow("Point cloud rendering");
    glewInit();
}

GLuint createProgram(string& log, GLuint vertShaderId, GLuint fragmentShaderId)
{
    log += "createProgram  (linker stage ) \n";
    if (vertShaderId > 0 && fragmentShaderId > 0){
        GLuint programId = glCreateProgram();
        GLuint err = glGetError();
        glAttachShader(programId, vertShaderId);
        glAttachShader(programId, fragmentShaderId);
        glLinkProgram(programId);
        int success = 0;
        glGetProgramiv(programId, GL_LINK_STATUS, &success);
        err = glGetError();

        if (success > 0){
            return programId;
        }
        else{
            int bufSize;
            glGetProgramiv(programId, GL_INFO_LOG_LENGTH, &bufSize);
            err = glGetError();
            if (bufSize > 0){
                char *pBuf = new char[bufSize];
                int slen;
                glGetProgramInfoLog(programId, bufSize, &slen, pBuf);
                err = glGetError();
                string compiler_log(pBuf);
                log += compiler_log;
            }
        }
    }
    return 0;
}

string loadShaderCode(string shaderName)
{
    ifstream shaderFile;
    shaderFile.open(shaderName);
    stringstream shaderStream;
    shaderStream << shaderFile.rdbuf();
    shaderFile.close();
    return shaderStream.str();
}

enum ShaderType
{
    Vertex,
    Fragment,
    Geometry,
    Tessellation,
    Compute
};

GLuint createShader(string shaderName, ShaderType shaderType, string& log)
{
    log += "createShader (compile) : " + shaderName + "\n";
    string extension = shaderType == ShaderType::Vertex ? ".vsh" : ".frag";
    string source = loadShaderCode(shaderName + extension);

    GLuint glShaderType = shaderType == ShaderType::Vertex ? GL_VERTEX_SHADER : GL_FRAGMENT_SHADER;
    GLuint shaderId = glCreateShader(glShaderType);

    if (shaderId > 0) {
        const char* c_str = source.c_str();
        glShaderSource(shaderId, 1, &c_str, nullptr);
        glCompileShader(shaderId);

        int success = 0;
        glGetShaderiv(shaderId, GL_COMPILE_STATUS, &success);
        if (success > 0) {
            return shaderId;
        }
        else {
            int bufSize = 0;
            glGetShaderiv(shaderId, GL_INFO_LOG_LENGTH, &bufSize);
            if (bufSize > 0) {
                int slen;
                char* pBuf = new char[bufSize];
                glGetShaderInfoLog(shaderId, bufSize, &slen, pBuf);
                string compiler_log(pBuf);
                log += compiler_log;
            }
        }
    }
    return 0;
}

int pointCount;
int bufferCount;
int* bufferSizes;
int bufferSizesMax;
unsigned int* bufferCodes;
float** pointPositions;
byte** pointRGBs;
double* pOffsetpunkt;
double* pbox;
Kbvis::Box3d* pBox3d;
GLuint* pointBuffers = nullptr;
GLuint* rgbBuffers = nullptr;
GLuint* kernelBuffers = nullptr;
GLuint pointCloudShader = 0;

void loadLasFile()
{
    pointCount = OpenLasFiles(new char* [1]{ "test.las" }, 1);
    //pointCount = OpenLasFiles(new char* [1]{ "flower.las" }, 1);
    //pointCount = OpenLasFiles(new char*[1]{ "statue.las" }, 1);
    //pointCount = OpenLasFiles(new char*[1]{ "towerComplete.las" }, 1);

    pointPositions = ReadPoints(&bufferCount, &bufferSizes, &bufferCodes, PointAttributes::PointAttribute::RGB, 0);
    pointRGBs = GetPointRGBs();

    cerr << "\n***************** " << bufferCount << " buffers ***************************\n";

    bufferSizesMax = 0;

    for (int i = 0; i < bufferCount; i++){
        float* pts = pointPositions[i];
        byte* prgb = pointRGBs != nullptr ? pointRGBs[i] : nullptr;
        bufferSizesMax = max(bufferSizesMax, bufferSizes[i]);

        cerr << "******************* " << i << " buffer size = " << bufferSizes[i] << endl;

        for (int j = 0; j < 2; j++) {
            cerr << *pts++ << " ";
            cerr << *pts++ << " ";
            cerr << *pts++ << " ";
            if (prgb != nullptr) {
                cerr << int(*prgb++) << " ";
                cerr << int(*prgb++) << " ";
                cerr << int(*prgb++) << " ";
            }
            cerr << endl;
        }
    }

    pOffsetpunkt = GetOffsetPoint();
    pbox = GetBoundingBox();
    pBox3d = GetBoundingBox3d();
    cerr << "\nBounding Box = " << pbox[0] << " " << pbox[1] << " " << pbox[2] << " " << pbox[3] << " " << pbox[4] << " " << pbox[5] << endl;
    double* poff = GetOffsetPoint();
    cerr << "\nOffsetPoint = " << poff[0] << " " << poff[1] << " " << poff[2] << endl;

    //CloseLasFiles();
}

GLuint knnBuffer, cellBuffer, kernelVerticesBuffer, kMeansBuffer;
GLfloat modelViewMatrix[16];
GLfloat projectionMatrix[16];
float m_FilterGridSize;
int cellResolutionX;
int cellResolutionY;
int cellResolutionZ;
int histogramPrecision = 128;
int kernelMaxK;
int kernelK = 20;
float stddevMult = 0.f;
long int maxBufferAlloc = 1024 * 1024 * 1024;
long int cellBufferSize = 0, knnBufferSize = 0, kernelVerticesBufferSize = 0, kMeansBufferSize = 0;

const GLuint locProjectionMatrix = 0;
const GLuint locModelViewMatrix = 1;
const GLuint locPointScale = 2;
const GLuint locBoundingBoxMin = 3;
const GLuint locBoundingBoxSize = 4;
const GLuint locFilterPass = 5;
const GLuint locFilterResolution = 6;
const GLuint locFilterTolerance = 7;
const GLuint locFilterMaxKN = 8;

const GLuint locPointCenter = 0;
const GLuint locPointColor = 1;
const GLuint locPointRgb = 2;

enum KernelType
{
    KERNEL_NONE = 0,
    KERNEL_KMEANS = 1
};
enum KernelPass
{
    KERNEL_PASS_NONE = 0,
    KERNEL_PASS_KMEANS_OUTLIER_PARTITION = 1,
    KERNEL_PASS_KMEANS_OUTLIER_KNN = 2,
    KERNEL_PASS_KMEANS_OUTLIER_MEAN = 3,
    KERNEL_PASS_KMEANS_OUTLIER_FILTER = 4
};
KernelType filterType = KERNEL_NONE;

void loadPointCloud()
{
    pointBuffers = new GLuint[bufferCount];
    rgbBuffers = new GLuint[bufferCount];
    glGenBuffers(bufferCount, pointBuffers);
    glGenBuffers(bufferCount, rgbBuffers);
    for (int i = 0; i < bufferCount; ++i) {
        glBindBuffer(GL_ARRAY_BUFFER, pointBuffers[i]);
        glBufferData(GL_ARRAY_BUFFER, bufferSizes[i] * 3 * sizeof(float), pointPositions[i], GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, rgbBuffers[i]);
        glBufferData(GL_ARRAY_BUFFER, bufferSizes[i] * 3, pointRGBs[i], GL_DYNAMIC_DRAW);
    }

    string log = "";
    GLuint vertexShaderId = createShader("pointCloudShader", ShaderType::Vertex, log);
    GLuint fragShaderId = createShader("pointCloudShader", ShaderType::Fragment, log);
    if (vertexShaderId > 0 && fragShaderId > 0) {
        pointCloudShader = createProgram(log, vertexShaderId, fragShaderId);
        cerr << log;
    }
    else {
        cerr << log;
    }

    kernelBuffers = new GLuint[4];
    glGenBuffers(4, kernelBuffers);
    cellBuffer = kernelBuffers[0];
    kernelVerticesBuffer = kernelBuffers[1];
    knnBuffer = kernelBuffers[2];
    kMeansBuffer = kernelBuffers[3];
}

void RunKmeansKernel(long int maxBufferSize)
{
    glDisable(GL_DEPTH_TEST);
    glUniform3f(locBoundingBoxMin, float(pBox3d->_min.x - pOffsetpunkt[0]), float(pBox3d->_min.y - pOffsetpunkt[1]), float(pBox3d->_min.z - pOffsetpunkt[2]));
    glUniform3f(locBoundingBoxSize, float(pBox3d->Width()), float(pBox3d->Height()), float(pBox3d->Thickness()));
    m_FilterGridSize = pow(0.5f * pBox3d->Width() * pBox3d->Height() * pBox3d->Thickness() * kernelK / float(pointCount), 1.0 / 3.0);
    cellResolutionX = int(ceil(pBox3d->Width() / m_FilterGridSize));
    cellResolutionY = int(ceil(pBox3d->Height() / m_FilterGridSize));
    cellResolutionZ = int(ceil(pBox3d->Thickness() / m_FilterGridSize));
    glUniform3i(locFilterResolution, cellResolutionX, cellResolutionY, cellResolutionZ);
    glUniform3f(locFilterTolerance, kernelK, stddevMult, sqrt(2.f * m_FilterGridSize * m_FilterGridSize));
    kernelMaxK = min(64, int(maxBufferSize / (cellResolutionX * cellResolutionY * cellResolutionZ)));
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, knnBuffer);
    if (knnBufferSize < bufferSizesMax * histogramPrecision) {
        knnBufferSize = bufferSizesMax * histogramPrecision;
        glBufferData(GL_SHADER_STORAGE_BUFFER, knnBufferSize * sizeof(unsigned int), 0, GL_DYNAMIC_COPY);
    }
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, cellBuffer);
    if (cellBufferSize < cellResolutionX * cellResolutionY * cellResolutionZ * kernelMaxK) {
        cellBufferSize = cellResolutionX * cellResolutionY * cellResolutionZ * kernelMaxK;
        glBufferData(GL_SHADER_STORAGE_BUFFER, cellBufferSize * sizeof(unsigned int), 0, GL_DYNAMIC_COPY);
    }
    glClearBufferData(GL_SHADER_STORAGE_BUFFER, GL_R32UI, GL_RED, GL_UNSIGNED_INT, 0);
    glBindBufferRange(GL_SHADER_STORAGE_BUFFER, 1, cellBuffer, 0, cellBufferSize * sizeof(unsigned int));
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, kernelVerticesBuffer);
    if (kernelVerticesBufferSize < pointCount * 3) {
        kernelVerticesBufferSize = pointCount * 3;
        glBufferData(GL_SHADER_STORAGE_BUFFER, kernelVerticesBufferSize * sizeof(float), 0, GL_DYNAMIC_COPY);
    }
    glBindBufferRange(GL_SHADER_STORAGE_BUFFER, 3, kernelVerticesBuffer, 0, kernelVerticesBufferSize * sizeof(float));
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, kMeansBuffer);
    if (kMeansBufferSize < pointCount + 4) {
        kMeansBufferSize = pointCount + 4;
        glBufferData(GL_SHADER_STORAGE_BUFFER, kMeansBufferSize * sizeof(unsigned int), 0, GL_DYNAMIC_COPY);
    }
    glClearBufferData(GL_SHADER_STORAGE_BUFFER, GL_R32UI, GL_RED, GL_UNSIGNED_INT, 0);
    glBindBufferRange(GL_SHADER_STORAGE_BUFFER, 5, kMeansBuffer, 0, kMeansBufferSize * sizeof(unsigned int));
    glEnableVertexAttribArray(locPointCenter);
    int pointsDrawn = 0, pointPositionOffset = 0;
    for (int bufferIndex = 0; bufferIndex < bufferCount; bufferIndex++) {
        int pointDrawCount = bufferSizes[bufferIndex];
        if (pointDrawCount == 0) continue;
        glBindBuffer(GL_ARRAY_BUFFER, pointBuffers[bufferIndex]);
        glVertexAttribPointer(locPointCenter, 3, GL_FLOAT, false, 3 * sizeof(float), 0);
        glUniform3i(locFilterMaxKN, kernelMaxK, pointCount, pointsDrawn);
        glUniform2i(locFilterPass, KERNEL_PASS_KMEANS_OUTLIER_PARTITION, histogramPrecision);
        glDrawArrays(GL_POINTS, 0, pointDrawCount);
        glMemoryBarrier(0x00002000);
        pointsDrawn += pointDrawCount;
        pointPositionOffset += bufferSizes[bufferIndex];
    }
    pointsDrawn = 0;
    pointPositionOffset = 0;
    for (int bufferIndex = 0; bufferIndex < bufferCount; bufferIndex++) {
        int pointDrawCount = bufferSizes[bufferIndex];
        if (pointDrawCount == 0) continue;
        glBindBuffer(GL_ARRAY_BUFFER, pointBuffers[bufferIndex]);
        glVertexAttribPointer(locPointCenter, 3, GL_FLOAT, false, 3 * sizeof(float), 0);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, knnBuffer);
        glClearBufferData(GL_SHADER_STORAGE_BUFFER, GL_R32UI, GL_RED, GL_UNSIGNED_INT, 0);
        glBindBufferRange(GL_SHADER_STORAGE_BUFFER, 4, knnBuffer, 0, knnBufferSize * sizeof(unsigned int));
        glUniform3i(locFilterMaxKN, kernelMaxK, pointCount, pointsDrawn);
        for (int pass = KERNEL_PASS_KMEANS_OUTLIER_KNN; pass <= KERNEL_PASS_KMEANS_OUTLIER_MEAN; ++pass) {
            glUniform2i(locFilterPass, pass, histogramPrecision);
            glDrawArrays(GL_POINTS, 0, pointDrawCount);
            glMemoryBarrier(0x00002000);
        }
        pointsDrawn += pointDrawCount;
        pointPositionOffset += bufferSizes[bufferIndex];
    }
    assert(GL_NO_ERROR == glGetError());
}

void renderPointCloud()
{
    if (pointBuffers == nullptr){
        loadPointCloud();
    }

    int maxBufferAlloc = 4096 * 256 * 256;

    glUseProgram(pointCloudShader);
    glEnable(GL_POINT_SPRITE);
    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
    glUniformMatrix4fv(locProjectionMatrix, 1, false, projectionMatrix);
    glUniformMatrix4fv(locModelViewMatrix, 1, false, modelViewMatrix);

    if (filterType == KERNEL_KMEANS){
        RunKmeansKernel(maxBufferAlloc);
    }

    glEnable(GL_DEPTH_TEST);

    int pointsDrawn = 0;

    for (int b = 0; b < bufferCount; b++){
        
        int pointDrawCount = bufferSizes[b];

        if (filterType == KERNEL_KMEANS) {
            glUniform3i(locFilterMaxKN, 0, pointCount, pointsDrawn);
            glUniform3f(locFilterTolerance, 0, stddevMult, 0);
            glUniform2i(locFilterPass, KERNEL_PASS_KMEANS_OUTLIER_FILTER, 0);
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, kMeansBuffer);
            glBindBufferRange(GL_SHADER_STORAGE_BUFFER, 5, kMeansBuffer, 0, kMeansBufferSize * sizeof(unsigned int));
            assert(GL_NO_ERROR == glGetError());
        }
        else {
            glUniform2i(locFilterPass, KERNEL_PASS_NONE, 0);
        }

        glBindBuffer(GL_ARRAY_BUFFER, rgbBuffers[b]);
        glEnableVertexAttribArray(locPointRgb);
        glVertexAttribPointer(locPointRgb, 3, GL_UNSIGNED_BYTE, true, 3, 0);
        glEnableVertexAttribArray(locPointCenter);
        glBindBuffer(GL_ARRAY_BUFFER, pointBuffers[b]);
        glVertexAttribPointer(locPointCenter, 3, GL_FLOAT, false, 3 * sizeof(float), 0);

        glDrawArrays(GL_POINTS, 0, pointDrawCount);

        glDisableVertexAttribArray(locPointCenter);
        glDisableVertexAttribArray(locPointRgb);

        pointsDrawn += pointDrawCount;
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glDisable(GL_POINT_SPRITE);
    glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
}

void display()
{
	sdkStartTimer(&timer);

    double size = 0.5 * sqrt(pBox3d->Width() * pBox3d->Width() + pBox3d->Height() * pBox3d->Height() + pBox3d->Thickness() * pBox3d->Thickness());
	double xmin = pbox[0] - pOffsetpunkt[0];
	double xmax = pbox[3] - pOffsetpunkt[0];
	double ymin = pbox[1] - pOffsetpunkt[1];
	double ymax = pbox[4] - pOffsetpunkt[1];
	double zmin = pbox[2] - pOffsetpunkt[2];
	double zmax = pbox[5] - pOffsetpunkt[2];
	double xmid = (xmin + xmax) / 2.0;
	double ymid = (ymin + ymax) / 2.0;
	double zmid = (zmin + zmax) / 2.0;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
    double scaledSize = scale * size;
    glOrtho(xmid - scaledSize, xmid + scaledSize, ymid - scaledSize, ymid + scaledSize, zmid - size, zmid + size);
	glGetFloatv(GL_PROJECTION_MATRIX, projectionMatrix);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
    glTranslatef(xmid, ymid, zmid);
	glRotatef(-viewRotation.x, 1.0, 0.0, 0.0);
	glRotatef(-viewRotation.y, 0.0, 1.0, 0.0);
    glTranslatef(-xmid, -ymid, -zmid);
	glGetFloatv(GL_MODELVIEW_MATRIX, modelViewMatrix);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    renderPointCloud();

    glutSwapBuffers();
    glutReportErrors();

    sdkStopTimer(&timer);
    computeFPS();
}

void keyboard(unsigned char key, int x, int y)
{
    switch (key) {
    case 27:
        glutDestroyWindow(glutGetWindow());
        return;
        break;
    case 'f':
        filterType = filterType == KERNEL_NONE ? KERNEL_KMEANS : KERNEL_NONE;
        break;
    case '+':
        scale += 0.01;
        break;
    case '-':
        scale -= 0.01;
        break;
    case ']':
        stddevMult += 0.1;
        break;
    case '[':
        stddevMult -= 0.1;
        break;
    case ';':
        break;
    case '\'':
        break;
    case '.':
        kernelK += 1;
        break;
    case ',':
        kernelK -= 1;
        break;
    case 'r':
        scale = 1;
        stddevMult = 0;
        kernelK = 10;
        viewRotation.x = viewRotation.y = viewRotation.z = 0;
        break;
    default:
        break;
    }

    glutPostRedisplay();
}

int main(int argc, char** argv)
{
    initGL(&argc, argv);

    loadLasFile();

    sdkCreateTimer(&timer);

    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutReshapeFunc(reshape);
    glutIdleFunc(idle);
    glutCloseFunc(cleanup);
    glutMainLoop();
}
