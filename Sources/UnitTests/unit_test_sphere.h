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
    Coordinate x1, x2, x3, x4;

    x1.set(95.5/Rad2Deg, 0.0/Rad2Deg);
    x2.set(89.5/Rad2Deg, 0.0/Rad2Deg);
    x3.set(89.75/Rad2Deg, -0.249998/Rad2Deg);
    x4.set(90.25/Rad2Deg, -0.249998/Rad2Deg);

    cout << Sphere::isIntersect(x1, x2, x3, x4) << endl;
}

#endif
