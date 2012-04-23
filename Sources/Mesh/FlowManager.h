#ifndef FlowManager_h
#define FlowManager_h

#include "MeshManager.h"
#include "Field.h"
#include "PolarRing.h"
#include "Velocity.h"
#include <string>

using std::string;

class FlowManager
{
public:
	FlowManager ();
	virtual ~FlowManager ();

    void init(const MeshManager &meshManager);

    void update(double *u, double *v);

    void getVelocity(const Coordinate &, const Location &,
                     TimeLevel, Velocity &, Velocity::Type) const;

    void output(const string &fileName) const;
#ifdef DEBUG
    void checkVelocityField(MeshManager &meshManager);
#endif

	Field u, v, w;
    PolarRingVelocity prv;

private:
    bool isInitialized;
};

#endif
