#ifndef unit_test_sphere_h
#define unit_test_sphere_h

#include "Sphere.h"

void test_orientation()
{
    Coordinate x1, x2, x3;
    OrientStatus orient;

    x3.set(0.0, 1.0/Rad2Deg);

    x1.set(355.0/Rad2Deg, 0.0);
    x2.set(5.0/Rad2Deg, 0.0);
    
    orient = Sphere::orient(x1, x2, x3);

    assert(orient == OrientLeft);

    x1.set(185.0/Rad2Deg, 0.0);
    x2.set(175.0/Rad2Deg, 0.0);

    orient = Sphere::orient(x1, x2, x3);

    assert(orient == OrientRight);
}

void test_intersection()
{
    Coordinate x1, x2, x;
    double lon1, lon2, lat;

#ifndef INTERSECTION_FAILURE_1
    x1.set(1.2280734796218382154, 0.66012992695828620615);
    x2.set(1.2458688154227666178, 0.66312169804405485252);
    lon1 = 1.2348204457859882;
    lon2 = 1.2435470920459597;
    lat = 0.66138792707153526;
#endif
#ifdef INTERSECTION_FAILURE_2
    x1.set(5.9189382174881215448, -0.56333636933501674005);
    x2.set(5.9881996873649860902, -0.4759259931534789434);
    lon1 = 5.9297561336507343;
    lon2 = 5.9384827799107063;
    lat = -0.54825578165140443;
#endif
    
    Sphere::calcIntersectLon(x1, x2,lon1, lon2, lat, x);
}

#endif
