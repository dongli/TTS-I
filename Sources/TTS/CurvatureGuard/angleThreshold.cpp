#ifndef angleThreshold_h
#define angleThreshold_h

//! Angle threshold to control the edge-splitting and edge-merging.
/*!
 The edge-splitting and edge-merging are the main components of curvature guard,
 so when to execute them is critical for catching curvature change well and
 correctly, and also for accomplishing balance between efficiency and performance.
 */

#include "CurvatureGuard.h"
#include "Edge.h"
#include "Sphere.h"
#include "ThresholdFunction.h"

void CurvatureGuard::calcAngleThreshold(Edge *edge, double &a0)
{
    static const double L[] = {
        0.01/Rad2Deg*Sphere::radius,
        0.5/Rad2Deg*Sphere::radius,
        1.0/Rad2Deg*Sphere::radius,
        4.0/Rad2Deg*Sphere::radius
    };
    static const double A[] = {
        180.0/Rad2Deg,
        60.0/Rad2Deg,
        30.0/Rad2Deg,
        1.0/Rad2Deg
    };
    double l = edge->getLength();
    a0 = piecewiseCubicthreshold(4, L, A, l);
}

void CurvatureGuard::calcAngleThreshold(Edge *edge1, Edge *edge2, double &a0)
{
    double A1, A2;
    CurvatureGuard::calcAngleThreshold(edge1, A1);
    CurvatureGuard::calcAngleThreshold(edge2, A2);
    // TODO: Choose a better weight function.
//    a0 = fmin(A1, A2);
    a0 = (A1+A2)*0.5;
}

void CurvatureGuard::relaxAngleThreshold(Edge *edge1, Edge *edge2, double &a0)
{
    static const double L[] = {
        0.01/Rad2Deg*Sphere::radius,
        0.5/Rad2Deg*Sphere::radius,
        1.0/Rad2Deg*Sphere::radius,
        4.0/Rad2Deg*Sphere::radius
    };
    static const double R[] = {
        1.0,
        0.8,
        0.5,
        0.2
    };
    double l = fmax(edge1->getLength(), edge2->getLength());
    a0 *= piecewiseCubicthreshold(4, L, R, l);
}

#endif