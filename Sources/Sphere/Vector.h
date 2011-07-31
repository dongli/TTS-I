#ifndef _Vector_h_
#define _Vector_h_

#include <blitz/tinyvec.h>

using blitz::TinyVector;

typedef TinyVector<double, 3> Vector;

inline double norm(const Vector &vector)
{
    return sqrt(vector(0)*vector(0)+vector(1)*vector(1)+vector(2)*vector(2));
}

#endif