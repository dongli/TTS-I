#ifndef _FlowManager_h_
#define _FlowManager_h_

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

    void construct(const MeshManager &meshManager);

    void update(double *u, double *v);

    void getVelocity(const Coordinate &, const Location &,
                     TimeLevel, Velocity &, Velocity::Type) const;

    void output(const string &fileName) const;

	Field u, v, w;
    PolarRingVelocity prv;

private:
    bool isInitialized;
};

#endif
