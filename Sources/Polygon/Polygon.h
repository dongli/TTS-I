#ifndef _Polygon_h_
#define _Polygon_h_

#include "List.h"
#include "Edge.h"

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

private:
    bool isAreaSet;
    MultiTimeLevel<double, 2> area;
};

#endif
