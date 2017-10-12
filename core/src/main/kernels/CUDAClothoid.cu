#include "math.h"

__device__ void getCircleTrajectory(int nbPoint, float curvature, float orientation, float xRobot, float yRobot, float* xOutput, float* yOutput, float* orientationOutput, float* curvatureOutput)
{
  float radius = 1000.f / curvature;
  float xDelta = cosf(orientation + 3.1415f/2.f) * radius;
  float yDelta = sinf(orientation + 3.1415f/2.f) * radius;
  float xCenter = xRobot + xDelta;
  float yCenter = yRobot + yDelta;
  float angle = 0.02f * curvature;
  float cosv = cosf(angle);
  float sinv = sinf(angle);
  float tmp = xDelta;
  xDelta = cosv * xDelta - sinv * yDelta;
  yDelta = sinv * tmp + cosv * yDelta;
  *xOutput = xCenter - xDelta;
  *yOutput = yCenter - yDelta;
  *orientationOutput = orientation + (nbPoint+1) * angle;
  *curvatureOutput = curvature;
}

__device__ void getStraightTrajectory(int nbPoint, float orientation, float xRobot, float yRobot, float* xOutput, float* yOutput, float* orientationOutput, float* curvatureOutput)
{
  float cosv = cosf(orientation);
  float sinv = sinf(orientation);
  float d = (float)(nbPoint + 1) * 20.f;
  *xOutput = xRobot + d * cosv;
  *yOutput = yRobot + d * sinv;
  *curvatureOutput = 0;
  *orientationOutput = orientation;
}

__device__ void compute(int nbPoint, float* xUnitary, float* yUnitary, int tentacleSpeed, float tentacleSquaredRootSpeed, float tentacleInitialCurvature, int16_t tentaclePositive, int16_t tentacleBack, int16_t tentacleStop, float xRobot, float yRobot, float orientationRobot, float curvatureRobot, int16_t goingForwardRobot, float* xOutput, float* yOutput, float* orientationOutput, float* curvatureOutput, int16_t* goingForwardOutput)
{
  if(tentacleBack)
    orientationRobot += 3.1415f;

  if(tentacleBack || tentacleStop)
    curvatureRobot = tentacleInitialCurvature;

  *goingForwardOutput = tentacleBack != goingForwardRobot;
  if(tentacleSpeed == 0)
  {
    if(curvatureRobot < 0.0001f && curvatureRobot > -0.0001f)
      getStraightTrajectory(nbPoint, orientationRobot, xRobot, yRobot, xOutput, yOutput, orientationOutput, curvatureOutput);
    else
      getCircleTrajectory(nbPoint, curvatureRobot, orientationRobot, xRobot, yRobot, xOutput, yOutput, orientationOutput, curvatureOutput);
  }
  else
  {
    float coeff = 1.f / tentacleSquaredRootSpeed;
    float sDepart = curvatureRobot * coeff;
    if(tentacleSpeed < 0)
      sDepart = -sDepart;

    int pointDepart = (int) ((sDepart / 0.02f) + 500.f - 1.f + 0.5f);
    float orientationClotho = sDepart * sDepart;
    if(tentacleSpeed < 0)
      orientationClotho = -orientationClotho;
    float base = orientationRobot  - orientationClotho;
    float cosv = cosf(base);
    float sinv = sinf(base);
    sDepart += ((float)(nbPoint+1) * tentacleSquaredRootSpeed * 0.02f);

    *xOutput = (float)(xUnitary[(int)(pointDepart + (int)tentacleSquaredRootSpeed * (nbPoint + 1))] - xUnitary[pointDepart]) * coeff;
    *yOutput = (float)(yUnitary[(int)(pointDepart + (int)tentacleSquaredRootSpeed * (nbPoint + 1))] - yUnitary[pointDepart]) * coeff;
    if(tentacleSpeed < 0)
      *yOutput = -*yOutput;
    float tmp = *xOutput;
    *xOutput = cosv*(*xOutput) - sinv*(*yOutput) + xRobot;
    *yOutput = sinv*tmp + cosv*(*yOutput) + yRobot;
    *curvatureOutput = sDepart * tentacleSquaredRootSpeed;
    *orientationOutput = sDepart * sDepart;
    if(tentacleSpeed < 0)
    {
      *orientationOutput = -*orientationOutput;
      *curvatureOutput = -*curvatureOutput;
    }
    *orientationOutput += base;
  }
}

extern "C"
__global__ void kernelFunc(float* xUnitary, float* yUnitary, int* tentacleSpeed, float* tentacleSquaredRootSpeed, float* tentacleInitialCurvature, int16_t* tentaclePositive, int16_t* tentacleBack, int16_t* tentacleStop, float* xRobot, float* yRobot, float* orientationRobot, float* curvatureRobot, int16_t* goingForwardRobot, float* xOutput, float* yOutput, float* orientationOutput, float* curvatureOutput, int16_t* goingForwardOutput)
{
    int nbPoint = blockIdx.y;
    int tentaculeNumber = blockIdx.x;

    compute(nbPoint, xUnitary, yUnitary, tentacleSpeed[tentaculeNumber], tentacleSquaredRootSpeed[tentaculeNumber], tentacleInitialCurvature[tentaculeNumber], tentaclePositive[tentaculeNumber], tentacleBack[tentaculeNumber], tentacleStop[tentaculeNumber], xRobot[0], yRobot[0], orientationRobot[0], curvatureRobot[0], goingForwardRobot[0], &xOutput[5*tentaculeNumber + nbPoint], &yOutput[5*tentaculeNumber + nbPoint], &orientationOutput[5*tentaculeNumber + nbPoint], &curvatureOutput[5*tentaculeNumber + nbPoint], &goingForwardOutput[5*tentaculeNumber + nbPoint]);
}
