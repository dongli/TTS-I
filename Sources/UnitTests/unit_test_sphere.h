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
    Coordinate x1, x2;

//    x1.set(0.32305511776974543681, -0.77990009057896714229);
//    x2.set(0.3145368668335100093,  -0.76699899808209737184);
    
    x1.set(0.32305511776974543681, -0.77990009057896714229);
    x2.set(0.3155368668335100093,  -0.76699899808209737184);

    cout << "x1: ";
    cout << setw(20) << setprecision(15) << x1.getLon()*Rad2Deg;
    cout << setw(20) << setprecision(15) << x1.getLat()*Rad2Deg << endl;
    cout << "x2: ";
    cout << setw(20) << x2.getLon()*Rad2Deg;
    cout << setw(20) << x2.getLat()*Rad2Deg << endl;

    double lat = -0.77252278366962113;
    Coordinate x3, x4;

    cout << "lat: " << lat*Rad2Deg << endl;

    Sphere::calcIntersectLon(x1, x2, lat, x3, x4);

    cout << "x3: ";
    cout << setw(20) << x3.getLon()*Rad2Deg;
    cout << setw(20) << x3.getLat()*Rad2Deg << endl;
    cout << "x4: ";
    cout << setw(20) << x4.getLon()*Rad2Deg;
    cout << setw(20) << x4.getLat()*Rad2Deg << endl;
    
    cout << x1.getX() << ",";
    cout << x1.getY() << ",";
    cout << x1.getZ() << endl;
    cout << x2.getX() << ",";
    cout << x2.getY() << ",";
    cout << x2.getZ() << endl;
    cout << x3.getX() << ",";
    cout << x3.getY() << ",";
    cout << x3.getZ() << endl;
    cout << x4.getX() << ",";
    cout << x4.getY() << ",";
    cout << x4.getZ() << endl;
}

#endif
