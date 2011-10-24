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

    static void handleLinePolygon(PolygonManager &, Polygon *);
    static void handlePointPolygon(PolygonManager &, Polygon *);

    void dump(const string &fileName = "") const;

    List<EdgePointer> edgePointers;
#ifdef TTS_ONLINE
    vector<Tracer> tracers;
#endif

private:
    bool isAreaSet;
    MultiTimeLevel<double, 2> area;
};

#endif
