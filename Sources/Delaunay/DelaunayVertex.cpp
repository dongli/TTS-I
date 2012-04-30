#include "DelaunayVertex.hpp"

DelaunayVertex::DelaunayVertex()
{
    topology.DVT = this;
    pit.DVT = this;
    point = NULL;
    reinit();
}

DelaunayVertex::~DelaunayVertex()
{
}

void DelaunayVertex::reinit()
{
    inserted = false;
    topology.reinit();
    pit.reinit();
}

void DelaunayVertex::dump()
{
    cout << "Delaunay vertex (" << getID() << "):" << endl;
    topology.dump();
}