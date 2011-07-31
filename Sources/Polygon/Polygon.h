#ifndef _Polygon_h_
#define _Polygon_h_

#include "List.h"
#include "Edge.h"

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

    void dump() const;

    List<EdgePointer> edgePointers;

private:
    bool isAreaSet;
    MultiTimeLevel<double, 2> area;
};

#endif
