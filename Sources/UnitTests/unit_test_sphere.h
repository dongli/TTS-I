#include "Sphere.h"

BOOST_AUTO_TEST_SUITE(Sphere_Tests)

BOOST_AUTO_TEST_CASE(Test_Distance_Calculation)
{
    Coordinate x1, x2;

    x1.set(0.0, 0.0, 1.0);
    x2.set(PI05, 0.0, 1.0);
    double distance = Sphere::calcDistance(x1, x2);
    message.str("");
    message << "\nactual distance = " << setw(10) << distance;
    message << "\nexpect distance = " << setw(10) << PI05;
    BOOST_REQUIRE_MESSAGE(distance == PI05, message.str());
}

BOOST_AUTO_TEST_CASE(Test_Rotation)
{
    Coordinate xp, xo, xr;

    xp.set(PI, PI05);
    xo.set(PI*1.33, PI05-1.34);
    Sphere::rotate(xp, xo, xr);
    BOOST_REQUIRE(xo.getLon() == xr.getLon()+PI);
    message.str("");
    message << "\nxo.getLat() = " <<
        setw(30) << setprecision(22) << xo.getLat();
    message << "\nxr.getLat() = " <<
        setw(30) << setprecision(22) << xr.getLat();
    BOOST_REQUIRE_MESSAGE(fabs(xo.getLat()-xr.getLat()) < EPS, message.str());

    xp.set(PI, PI05);
    xo.set(PI*1.4323, PI05-2.4324);
    Sphere::rotate(xp, xo, xr);
    BOOST_REQUIRE(xo.getLon() == xr.getLon()+PI);
    message.str("");
    message << "\nxo.getLat() = " <<
        setw(30) << setprecision(22) << xo.getLat();
    message << "\nxr.getLat() = " <<
        setw(30) << setprecision(22) << xr.getLat();
    BOOST_REQUIRE_MESSAGE(fabs(xo.getLat()-xr.getLat()) < EPS, message.str());

    xp.set(PI, PI05);
    xr.set(PI*1.4323, PI05-2.4324);
    Sphere::inverseRotate(xp, xo, xr);
    message.str("");
    message << "\nxo.getLon() = " <<
        setw(30) << setprecision(22) << xo.getLon();
    message << "\nxr.getLon() = " <<
        setw(30) << setprecision(22) << xr.getLon();
    BOOST_REQUIRE_MESSAGE(fabs(xo.getLon()-xr.getLon()-PI), message.str());
    message.str("");
    message << "\nxo.getLat() = " <<
        setw(30) << setprecision(22) << xo.getLat();
    message << "\nxr.getLat() = " <<
        setw(30) << setprecision(22) << xr.getLat();
    BOOST_REQUIRE_MESSAGE(fabs(xo.getLat()-xr.getLat()) < EPS, message.str());

}

BOOST_AUTO_TEST_SUITE_END()
