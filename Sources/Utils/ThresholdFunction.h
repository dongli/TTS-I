#ifndef ThresholdFunction_h
#define ThresholdFunction_h

double piecewiseLinearThreshold(int n, const double *xs, const double *ys, double x)
{
    if (x < xs[0])
        return ys[0];
    else if (x >= xs[n-1])
        return ys[n-1];
    else
        for (int i = 0; i < n-1; ++i)
            if (x >= xs[i] && x < xs[i+1])
                return (x-xs[i])*(ys[i+1]-ys[i])/(xs[i+1]-xs[i])+ys[i];
}

double piecewiseCubicThreshold(int n, const double *X, const double *Y, double x)
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

#endif
