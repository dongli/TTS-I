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

    void calcArea();
    double getArea(TimeLevel timeLevel = NewTimeLevel) const {
        return area.get(timeLevel);
    }

    void dump(const char *fileName) const;
    void dump() const;
    void dump(std::ostream *output) const;

    List<EdgePointer> edgePointers;
#ifdef TTS_ONLINE
    vector<Tracer> tracers;
#endif

private:
    bool isAreaSet;
    MultiTimeLevel<double, 2> area;
};

#endif
