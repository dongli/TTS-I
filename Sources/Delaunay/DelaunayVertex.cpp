#include "DelaunayVertex.h"

DelaunayVertex::DelaunayVertex()
{
    topology.DVT = this;
    pit.DVT = this;
    reinit();
}

DelaunayVertex::~DelaunayVertex()
{
}

void DelaunayVertex::reinit()
{
    point = NULL;
    inserted = false;
    topology.reinit();
    pit.reinit();
}
