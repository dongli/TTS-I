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

#endif
