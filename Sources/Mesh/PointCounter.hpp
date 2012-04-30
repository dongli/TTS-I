#ifndef PointCounter_h
#define PointCounter_h

#include <blitz/array.h>
#include <string>

using blitz::Array;
using std::string;

#include "Location.hpp"
#include "Point.hpp"
#include "RLLMesh.hpp"

class PointCounter
{
public:
    PointCounter();
    virtual ~PointCounter();

    void init(const Array<double, 1> &lon, const Array<double, 1> &lat,
              int numSubLon, int numSubLat);

    void reset();

    void count(const Location &loc, Point *point);

    void output(const string &fileName) const;

    enum MeshType {
        Bound = 0, Center = 1
    };

    int numSubLon, numSubLat;
    RLLMesh mesh[2];
    Array<int, 3> counters;
    Array<Array<Point *, 1>, 3> points;
};

#endif
