#ifndef Vector_h
#define Vector_h

#include <blitz/tinyvec.h>

using blitz::TinyVector;

typedef TinyVector<double, 3> Vector;

inline double norm(const Vector &vector)
{
    return sqrt(vector(0)*vector(0)+vector(1)*vector(1)+vector(2)*vector(2));
}

inline Vector norm_cross(const Vector &vector1, const Vector &vector2)
{
    Vector ans = cross(vector1, vector2);
    ans /= norm(ans);
    return ans;
}

inline Vector operator-(const Vector &vector1, const Vector &vector2)
{
    Vector ans;
    for (int i = 0; i < 3; ++i)
        ans(i) = vector1(i)-vector2(i);
    return ans;
}

#endif