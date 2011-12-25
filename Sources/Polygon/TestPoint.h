#ifndef TestPoint_h
#define TestPoint_h

#include "Vertex.h"

class MeshManager;

class TestPoint : public Vertex
{
public:
    TestPoint();
    virtual ~TestPoint();

    void reinit();
    void reset(MeshManager &);
    void reset(MeshManager &, const FlowManager &, const Coordinate &);

    void calcAngle();
    double getAngle() const { return angle; }

    void calcOrient();
    OrientStatus getOrient() const { return orient; }

    TestPoint &operator=(const TestPoint &);
    TestPoint &operator=(const Vertex &);

private:
    double angle;
    OrientStatus orient;
};

#endif
