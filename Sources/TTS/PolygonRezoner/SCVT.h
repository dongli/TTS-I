#ifndef SCVT_h
#define SCVT_h

#include <blitz/array.h>

using blitz::Array;

class DelaunayDriver;

namespace SCVT
{
    void init(int numLon, int numLat, const double *lon, const double *lat,
              int maxIteration = 10);

    Array<double, 2> &getDensityFunction();
    void outputDensityFunction(const char *fileName);

    void run(int numPoint, double *lon, double *lat);
    void run(int numPoint, DelaunayDriver &driver);
}

#endif
