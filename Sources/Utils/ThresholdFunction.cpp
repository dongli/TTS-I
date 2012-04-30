#include <cmath>
#include "ThresholdFunction.hpp"

double ThresholdFunction::piecewiseLinear(int n, const double *X,
                                          const double *Y, double x)
{
    if (x < X[0])
        return Y[0];
    else if (x >= X[n-1])
        return Y[n-1];
    else
        for (int i = 0; i < n-1; ++i)
            if (x >= X[i] && x < X[i+1])
                return (x-X[i])*(Y[i+1]-Y[i])/(X[i+1]-X[i])+Y[i];
}

double ThresholdFunction::piecewiseCubic(int n, const double *X,
                                         const double *Y, double x)
{
    if (x < X[0])
        return Y[0];
    else if (x >= X[n-1])
        return Y[n-1];
    else
        for (int i = 0; i < n-1; ++i)
            if (x >= X[i] && x < X[i+1]) {
                double dX = X[i+1]-X[i];
                double dY = Y[i+1]-Y[i];
                double t = (x-X[i])/dX;
                return dY*(4.0-3.0*t)*pow(t, 3.0)+Y[i];
            }
}