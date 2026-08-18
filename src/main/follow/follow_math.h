#pragma once

#include "follow/follow_types.h"

float followLimit(float input, float minValue, float maxValue);
float followAngleDiff(float a, float b);
void followRotatePoint(float x, float y, float angleRadians, float *newX, float *newY);
uint8_t followChecksum(uint8_t *data, int32_t size);
void followRotateBodyToReference(const float bodyVector[3], const float angles[3], float referenceVector[3]);
void followRotateReferenceToBody(const float referenceVector[3], const float angles[3], float bodyVector[3]);
void followAlphaBetaUpdate(followAlphaBetaFilter_t *filter, float measurement, float dt);
float followVectorDotProduct(float a[3], const float b[3]);
void followVectorCrossProduct(float result[3], float a[3], float b[3]);
bool followVectorNormalize(float v[3]);
float followCalculateVectorAngle(const float a[3], const float b[3]);
float followVectorLength(const float v[3]);
