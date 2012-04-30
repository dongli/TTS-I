#ifndef ThresholdFunction_h
#define ThresholdFunction_h

namespace ThresholdFunction {
    double piecewiseLinear(int n, const double *X, const double *Y, double x);

    double piecewiseCubic(int n, const double *X, const double *Y, double x);
}

#endif
