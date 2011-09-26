#ifndef unit_test_tracers_h
#define unit_test_tracers_h

#include "TracerManager.h"

void test_tracers(void)
{
    TracerManager tracerManager;

    tracerManager.init("polygons_128x60.nc");
    //tracerManager.registerTracer("tracer1.nc");
}

#endif