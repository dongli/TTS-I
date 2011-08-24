#include "Sphere.h"
#include "RandomNumber.h"

BOOST_AUTO_TEST_SUITE(Sphere_Tests)

BOOST_AUTO_TEST_CASE(Test_Distance_Calculation)
{
    Coordinate x1, x2;

    x1.set(0.0, 0.0, 1.0);
    x2.set(PI05, 0.0, 1.0);
    double distance = Sphere::calcDistance(x1, x2);
    sprintf(message, "\nactual distance = %20.10f"
                     "\nexpected distance = %20.10f",
            distance, PI05);
    BOOST_REQUIRE_MESSAGE(distance == PI05, message);
}

BOOST_AUTO_TEST_CASE(Test_Rotation)
{
    Coordinate xp, xo, xr;

    xp.set(PI, PI05);
    xo.set(PI*1.33, PI05-1.34);
    Sphere::rotate(xp, xo, xr);
    BOOST_REQUIRE(xo.getLon() == xr.getLon()+PI);
    sprintf(message, "\noriginal latitude = %20.15f"
                     "\nrotated latitude = %20.15f",
            xo.getLat(), xr.getLat());
    BOOST_REQUIRE_MESSAGE(fabs(xo.getLat()-xr.getLat()) < EPS, message);

    xp.set(PI, PI05);
    xo.set(PI*1.4323, PI05-2.4324);
    Sphere::rotate(xp, xo, xr);
    BOOST_REQUIRE(xo.getLon() == xr.getLon()+PI);
    sprintf(message, "\noriginal latitude = %20.15f"
                     "\nrotated latitude = %20.15f",
            xo.getLat(), xr.getLat());
    BOOST_REQUIRE_MESSAGE(fabs(xo.getLat()-xr.getLat()) < EPS, message);

    xp.set(PI, PI05);
    xr.set(PI*1.4323, PI05-2.4324);
    Sphere::inverseRotate(xp, xo, xr);
    sprintf(message, "\noriginal longitude = %20.15f"
                     "\nrotated longitude = %20.15f",
            xo.getLon(), xr.getLon());
    BOOST_REQUIRE_MESSAGE(fabs(xo.getLon()-xr.getLon()-PI), message);
    sprintf(message, "\noriginal latitude = %20.15f"
                     "\nrotated latitude = %20.15f",
            xo.getLat(), xr.getLat());
    BOOST_REQUIRE_MESSAGE(fabs(xo.getLat()-xr.getLat()) < EPS, message);

}

BOOST_AUTO_TEST_CASE(Test_Projection)
{
    Coordinate x1, x2, x3, x4;
    double distance;

    x1.set(35/Rad2Deg, 40/Rad2Deg);
    x2.set(110/Rad2Deg, 31/Rad2Deg);

    while (Sphere::project(x1, x2, x3, x4, distance) == false) {
        double lon, lat;
        lon = RandomNumber::getRandomNumber(0.0, PI2);
        lat = RandomNumber::getRandomNumber(-PI05, PI05);
        x3.set(lon, lat);
    }

    Vector vector1 = cross(x1.getCAR(), x2.getCAR());
    Vector vector2 = cross(x3.getCAR(), x4.getCAR());
    double tmp = dot(vector1, vector2);
    BOOST_REQUIRE(fabs(tmp) < EPS);
}

BOOST_AUTO_TEST_CASE(Test_Intersection_Latitude)
{
    Coordinate x1, x2, x3, x4, x5, x6, x2r, x5r;

    double lon1 = 0.0, lon2 = 0.0, lat1 = 0.0, lat2 = 0.0;

    while (lon1 == lon2) {
        lon1 = RandomNumber::getRandomNumber(0.0, PI2);
        lon2 = RandomNumber::getRandomNumber(0.0, PI2);
    }
    while (lat1 == lat2) {
        lat1 = RandomNumber::getRandomNumber(-PI05, PI05);
        lat2 = RandomNumber::getRandomNumber(-PI05, PI05);
    }

    assert(lon1 != lon2);
    assert(lat1 != lat2);

    double lon = RandomNumber::getRandomNumber(lon1, lon2);

    /*cout << "lon1 = " << setw(30) << setprecision(25) << lon1 << ";" << endl;
    cout << "lat1 = " << setw(30) << setprecision(25) << lat1 << ";" << endl;
    cout << "lon2 = " << setw(30) << setprecision(25) << lon2 << ";" << endl;
    cout << "lat2 = " << setw(30) << setprecision(25) << lat2 << ";" << endl;
    cout << "lon3 = " << setw(30) << setprecision(25) << lon3 << ";" << endl;*/

    x1.set(lon1, lat1);
    x2.set(lon2, lat2);
    x3.set(lon, PI05);
    x4.set(lon, 0.0);

    Sphere::calcIntersect(x1, x2, x3, x4, x5, x6);

    Sphere::rotate(x1, x2, x2r);
    Sphere::rotate(x1, x5, x5r);

    double dlon = fabs(x2r.getLon()-x5r.getLon());
    sprintf(message, "\nx1 = (%20.15f,%20.15f), (%15.10f,%15.10f,%15.10f)"
            "\nx2 = (%20.15f,%20.15f), (%15.10f,%15.10f,%15.10f)"
            "\nx5 = (%20.15f,%20.15f), (%15.10f,%15.10f,%15.10f)"
            "\nx6 = (%20.15f,%20.15f), (%15.10f,%15.10f,%15.10f)"
            "\ndlon = %20.15f",
            x1.getLon()*Rad2Deg, x1.getLat()*Rad2Deg, x1.getX(), x1.getY(), x1.getZ(),
            x2.getLon()*Rad2Deg, x2.getLat()*Rad2Deg, x2.getX(), x2.getY(), x2.getZ(),
            x5.getLon()*Rad2Deg, x5.getLat()*Rad2Deg, x5.getX(), x5.getY(), x5.getZ(),
            x6.getLon()*Rad2Deg, x6.getLat()*Rad2Deg, x6.getX(), x6.getY(), x6.getZ(),
            dlon*Rad2Deg);
    BOOST_REQUIRE_MESSAGE(dlon < EPS || fabs(dlon-PI) < EPS, message);

    Sphere::calcIntersectLat(x1, x2, lon, lat1, lat2);

    BOOST_REQUIRE(fabs(lat1-x5.getLat()) < EPS && fabs(lat2-x6.getLat()) < EPS);
}

BOOST_AUTO_TEST_CASE(Test_Intersection_Longitude)
{
    Coordinate x1, x2, x3, x2r, x3r;

    double lon1 = 0.0, lon2 = 0.0, lat1 = 0.0, lat2 = 0.0;

    while (lon1 == lon2) {
        lon1 = RandomNumber::getRandomNumber(0.0, PI2);
        lon2 = RandomNumber::getRandomNumber(0.0, PI2);
    }
    while (lat1 == lat2) {
        lat1 = RandomNumber::getRandomNumber(-PI05, PI05);
        lat2 = RandomNumber::getRandomNumber(-PI05, PI05);
    }

    assert(lon1 != lon2);
    assert(lat1 != lat2);

    double lon3_1, lon3_2;
    double lat3 = RandomNumber::getRandomNumber(lat1, lat2);

    /*cout << "lon1 = " << setw(30) << setprecision(25) << lon1 << ";" << endl;
    cout << "lat1 = " << setw(30) << setprecision(25) << lat1 << ";" << endl;
    cout << "lon2 = " << setw(30) << setprecision(25) << lon2 << ";" << endl;
    cout << "lat2 = " << setw(30) << setprecision(25) << lat2 << ";" << endl;
    cout << "lat3 = " << setw(30) << setprecision(25) << lat3 << ";" << endl;*/

    x1.set(lon1, lat1);
    x2.set(lon2, lat2);

    Sphere::calcIntersectLon(x1, x2, lat3, lon3_1, lon3_2);

    x3.set(lon3_1, lat3);

    Sphere::rotate(x1, x2, x2r);
    Sphere::rotate(x1, x3, x3r);

    sprintf(message, "\nx1 = (%20.15f,%20.15f), (%15.10f,%15.10f,%15.10f)"
            "\nx2 = (%20.15f,%20.15f), (%15.10f,%15.10f,%15.10f)"
            "\nx3 = (%20.15f,%20.15f), (%15.10f,%15.10f,%15.10f)"
            "\nrotated x2 longitude = %20.15f"
            "\nrotated x3 longitude = %20.15f"
            "\nThey should be the same!",
            x1.getLon()*Rad2Deg, x1.getLat()*Rad2Deg, x1.getX(), x1.getY(), x1.getZ(),
            x2.getLon()*Rad2Deg, x2.getLat()*Rad2Deg, x2.getX(), x2.getY(), x2.getZ(),
            x3.getLon()*Rad2Deg, x3.getLat()*Rad2Deg, x3.getX(), x3.getY(), x3.getZ(),
            x2r.getLon()*Rad2Deg, x3r.getLon()*Rad2Deg);
    double dlon = fabs(x2r.getLon()-x3r.getLon());
    BOOST_REQUIRE_MESSAGE(dlon < EPS || fabs(dlon-PI) < EPS, message);
}

BOOST_AUTO_TEST_SUITE_END()
