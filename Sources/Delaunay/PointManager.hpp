#ifndef PointManager_h
#define PointManager_h

#include "Point.hpp"

class PointPointer : public ListElement<PointPointer>
{
public:
    PointPointer() { ptr = NULL; }
    virtual ~PointPointer() {}
    
    void reinit() { ptr = NULL; }

    Point *ptr;
};

class PointManager
{
public:
    PointManager();
    virtual ~PointManager();
    
    void init(int numPoint, double *lon, double *lat);
    
    List<PointPointer> points;
};

#endif
