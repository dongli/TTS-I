#ifndef _Polygon_h_
#define _Polygon_h_

#include <vector>

using std::vector;

#include "List.h"
#include "Edge.h"
#ifdef TTS_ONLINE
#include "Tracer.h"
#endif

class PolygonManager;

class Polygon : public ListElement<Polygon>
{
public:
    Polygon();
    virtual ~Polygon();

    void reinit();
    void destroy();

    void removeEdge(EdgePointer *, PolygonManager &);

    void calcArea();
    double getArea(TimeLevel timeLevel = NewTimeLevel) const {
        return area.get(timeLevel);
    }
    bool isAreaSet() { return areaSet; }

    void calcCentroid();
    Coordinate &getCentroid() {
        return centroid;
    }

    void dump(const char *fileName) const;
    void dump() const;
    void dump(std::ostream *output) const;

#ifdef TTS_ONLINE
    void updateTracer(int tracerId);
    void updateTracers();
    void handoverTracers();
    void handoverTracers(Polygon *polygon, double percent);
    vector<Tracer> tracers;
#endif
    List<EdgePointer> edgePointers;

private:
    bool areaSet;
    MultiTimeLevel<double, 2> area;
    Coordinate centroid;
};

#endif
