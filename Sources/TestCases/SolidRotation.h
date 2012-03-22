#ifndef SolidRotation_h
#define SolidRotation_h

#include "TestCase.h"

class SolidRotation : public TestCase
{
public:
    SolidRotation();
    virtual ~SolidRotation();

    void calcVelocityField(FlowManager &);

    void calcInitCond(MeshManager &, MeshAdaptor &, TracerManager &);

    void calcSolution(Field &);

private:
    // rotation
    double angleSpeed;   // angular speed
    double U0;
    double alpha;        // angle between rotation axis and pole axis
    Coordinate axisPole;

    // cosine hill
    Coordinate C0;  // cosine hill center
    Coordinate CR0; // rotated center
    double R;       // cosine hill radius
    double H0;      // cosine hill amplitude
};

#endif
