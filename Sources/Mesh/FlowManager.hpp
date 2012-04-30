#ifndef FlowManager_h
#define FlowManager_h

#include "MeshManager.hpp"
#include "Field.hpp"
#include "PolarRing.hpp"
#include "Velocity.hpp"
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
