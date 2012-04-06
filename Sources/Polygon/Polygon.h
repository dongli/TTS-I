#ifndef Polygon_h
#define Polygon_h

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

    void dump(const char *fileName, TimeLevel timeLevel = NewTimeLevel) const;
    void dump() const;
    void dump(std::ostream *output, TimeLevel timeLevel = NewTimeLevel) const;

#ifdef TTS_ONLINE
    void updateTracer(int tracerId);
    void updateTracers();
    void handoverTracers();
    void handoverTracers(Polygon *polygon, double percent);
    vector<Tracer> tracers;
    double tracerDiff;
#endif
    List<EdgePointer> edgePointers;

private:
    bool areaSet;
    MultiTimeLevel<double, 2> area;
};

#endif
