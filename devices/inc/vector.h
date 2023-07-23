#ifndef __VECTOR_H
#define __VECTOR_H

#include "system.h"

typedef struct vector
{
  float x, y, z;
} vector;

void vector_cross(const vector *a, const vector *b, vector *out);
float vector_dot(const vector *a,const vector *b);
void vector_normalize(vector *a);

#endif
