#include <iostream>
#include <iomanip>
#include <netcdfcpp.h>

#include "ReportMacros.h"
#include "Constants.h"
#include "unit_test_fixtures.h"
#include "unit_test_sphere.h"
#include "unit_test_mesh.h"
#include "unit_test_testcase.h"
#include "unit_test_tracers.h"

using namespace std;

// main entrance of unit tests
int main(void)
{
    test_orientation();
    test_checkLocation();
    //test_TestCase<Deformation>(5.0/600.0, 1);
    //test_tracers();
}