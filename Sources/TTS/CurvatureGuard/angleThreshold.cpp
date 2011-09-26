#ifndef angleThreshold_h
#define angleThreshold_h

//! Angle threshold to control the edge-splitting and edge-merging.
/*!
 The edge-splitting and edge-merging are the main components of curvature guard,
 so when to execute them is critical for catching curvature change well and
 correctly, and also for accomplishing balance between efficiency and performance.
 */

#include "Edge.h"
#include "Sphere.h"
#include "CurvatureGuard.h"

inline double CurvatureGuard::angleThreshold(Edge *edge)
{
    static double A0 = 0.05/Rad2Deg;
    static double A1 = 60.0/Rad2Deg;
    static double L0 = 0.001/Rad2Deg*Sphere::radius;
    static double L1 = 1.0/Rad2Deg*Sphere::radius;
    static double dA = A1-A0;
    static double dL = L1-L0;
    
    double length = edge->getLength();
    if (length > L0 && length < L1) {
        double t = 1.0-(length-L0)/dL;
        return dA*(4.0-3.0*t)*pow(t, 3.0)+A0;
    } else if (length <= L0) {
        return A1;
    } else {
        return A0;
    }
}

inline double CurvatureGuard::angleThreshold(Edge *edge1, Edge *edge2)
{
    double A1 = CurvatureGuard::angleThreshold(edge1);
    double A2 = CurvatureGuard::angleThreshold(edge2);
    //return (A1+A2)*0.5;
    return fmin(A1, A2);
}

#endif