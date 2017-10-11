extern "C"
void getCircleTrajectory(int nbPoint, float curvature, float orientation, float xRobot, float yRobot, float* xOutput, float* yOutput, float* orientationOutput, float* curvatureOutput)
{
  float radius = 1000.f / curvature;
  float xDelta = cos(orientation + 3.1415f/2.f) * radius;
  float yDelta = sin(orientation + 3.1415f/2) * radius;
  float xCenter = xRobot + xDelta;
  float yCenter = yRobot + yDelta;
  float angle = (nbPoint+1) * 0.02f * curvature;
  float cosv = cos(angle);
  float sinv = sin(angle);
  float tmp = xDelta;
  xDelta = cosv * xDelta - sinv * yDelta;
  yDelta = sinv * tmp + cosv * yDelta;
  *xOutput = xCenter - xDelta;
  *yOutput = yCenter - yDelta;
  *orientationOutput = orientation + angle;
  *curvatureOutput = curvature;
}

extern "C"
void getStraightTrajectory(int nbPoint, float orientation, float xRobot, float yRobot, float* xOutput, float* yOutput, float* orientationOutput, float* curvatureOutput)
{
  float cosv = cos(orientation);
  float sinv = sin(orientation);
  float d = (nbPoint + 1) * 0.02f;
  *xOutput = xRobot + d * cosv;
  *yOutput = yRobot + d * sinv;
  *curvatureOutput = 0;
  *orientationOutput = orientation;
}


extern "C"
void compute(int nbPoint, float[] xUnitary, float[] yUnitary, int tentacleSpeed, float tentacleSquaredRootSpeed, float tentacleInitialCurvature, short tentaclePositive, short tentacleBack, short tentacleStop, float xRobot, float yRobot, float orientationRobot, float curvatureRobot, short goingForwardRobot, float* xOutput, float* yOutput, float* orientationOutput, float* curvatureOutput, short* goingForwardOutput)
{
  if(tentacleBack)
    orientationRobot += 3.1415f;

  if(tentacleBack || tentacleStop)
    curvatureRobot = tentacleInitialCurvature;

  *goingForwardOutput = tentacleBack != goingForwardRobot;
  if(tentacleSpeed == 0)
  {
    if(curvatureRobot == 0)
      getStraightTrajectory(nbPoint, orientationRobot, yRobot, yRobot, xOutput, yOutput, orientationOutput, curvatureOutput);
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
    float cosv = cos(base);
    float sinv = sin(base);
    sDepart += (nbPoint+1) * tentacleSquaredRootSpeed * 0.02f;

    xOutput = (xUnitary[pointDepart + tentacleSquaredRootSpeed * (nbPoint + 1)] - xUnitary[pointDepart]) * coeff;
    yOutput = (yUnitary[pointDepart + tentacleSquaredRootSpeed * (nbPoint + 1)] - yUnitary[pointDepart]) * coeff;
    if(tentacleSpeed < 0)
      *yOutput = -*yOutput;
    float tmp = *xOutput;
    *xOutput = cosv*(*xOutput) - sinv*(*yOutput) + xRobot;
    *yOutput = sinv*tmp + cosv*(*yOutput) + yRobot;
    *curvatureOutput = sDepart * tentacleSquaredRootSpeed;
    *orientationOutput = sDepart * sDepart;
    if(tentacleSpeed < 0)
      *orientationOutput = -*orientationOutput;
    *orientationOutput += base;
  }
}

extern "C"
__global__ void kernelFunc(float* xUnitary, float* yUnitary, int* tentacleSpeed, float* tentacleSquaredRootSpeed, float* tentacleInitialCurvature, short* tentaclePositive, short* tentacleBack, short* tentacleStop, float* xRobot, float* yRobot, float* orientationRobot, float* curvatureRobot, short* goingForwardRobot, float* xOutput, float* yOutput, float* orientationOutput, float* curvatureOutput, short* goingForwardOutput)
{
    nbPoint = blockIdx.y;

    compute(nbPoint, xUnitary, yUnitary, tentacleSpeed[0], tentacleSquaredRootSpeed[0], tentacleInitialCurvature[0], tentacleInitialCurvature[0], tentaclePositive[0], tentacleBack[0], tentacleStop[0], xRobot[0], yRobot[0], orientationRobot[0], curvatureRobot[0], goingForwardRobot[0], &xOutput[blockIdx.x * 5 + nbPoint], &yOutput[blockIdx.x * 5 + nbPoint], &orientationOutput[blockIdx.x * 5 + nbPoint], &curvatureOutput[blockIdx.x * 5 + nbPoint], &goingForwardOutput[blockIdx.x * 5 + nbPoint])

}