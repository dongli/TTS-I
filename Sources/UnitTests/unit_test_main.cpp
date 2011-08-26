#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#include "ReportMacros.h"
#include "Constants.h"
#include <boost/test/unit_test.hpp>
#include <iostream>
#include <iomanip>

using namespace std;

char message[255];

#include "unit_test_fixtures.h"
#include "unit_test_memory.h"
#include "unit_test_list.h"
#include "unit_test_mesh.h"
#include "unit_test_field.h"
#include "unit_test_flow.h"
#include "unit_test_sphere.h"
#include "unit_test_testcase.h"
#include "unit_test_polygon.h"
