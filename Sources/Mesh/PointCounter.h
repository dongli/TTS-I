#ifndef _PointCounter_h_
#define _PointCounter_h_

#include "Location.h"
#include "Point.h"
#include <blitz/array.h>
#include <string>

using blitz::Array;
using std::string;

class PointCounter
{
public:
    PointCounter();
    virtual ~PointCounter();

    void construct(const Array<double, 1> &lon,
                   const Array<double, 1> &lat,
                   int numSubLon, int numSubLat);

    void reset();

    void count(const Location &loc, Point *point);

    void output(const string &fileName) const;

    int numSubLon, numSubLat;
    Array<double, 1> lonBnds, latBnds;
    Array<int, 3> counters;
    Array<Array<Point *, 1>, 3> points;
};

#endif
