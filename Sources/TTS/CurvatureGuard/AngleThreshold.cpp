#include "AngleThreshold.h"
#include "Edge.h"
#include "ThresholdFunction.h"
#include "ConfigTools.h"
#include "Sphere.h"

using namespace CurvatureGuard;

Array<double, 1> AngleThreshold::A;
Array<double, 1> AngleThreshold::L;
Array<double, 1> AngleThreshold::R;

void AngleThreshold::init()
{
    int numEdgeSection;
    ConfigTools::read("num_edge_section", numEdgeSection);
    L.resize(numEdgeSection+1);
    A.resize(L.size());
    R.resize(L.size());
    ConfigTools::read("edge_length", L.size(), L.data());
    ConfigTools::read("angle_threshold", A.size(), A.data());
    ConfigTools::read("relax_factor", R.size(), R.data());
    L *= Sphere::radius/Rad2Deg;
    A /= Rad2Deg;
}

void AngleThreshold::calc(Edge *edge, double &a)
{
    double l = edge->getLength();
    a = ThresholdFunction::piecewiseCubic(A.size(), L.data(), A.data(), l);
    // TEST: The motion of point at high latitudes is sometimes not that
    //       consistent. So some wild small spurious edges will be generated.
    //       Here we filte them out!
    if (edge->getEndPoint(FirstPoint)->getLocation().inDangerousZone ||
        edge->getEndPoint(SecondPoint)->getLocation().inDangerousZone) {
        a *= 5.0;
    }
}

void AngleThreshold::calc(Edge *edge1, Edge *edge2, double &a)
{
    double a1, a2;
    calc(edge1, a1);
    calc(edge2, a2);
    a = (a1+a2)*0.5;
}

void AngleThreshold::relax(Edge *edge1, Edge *edge2, double &a)
{
    double l = fmax(edge1->getLength(), edge2->getLength());
    double r = ThresholdFunction::piecewiseCubic(R.size(), L.data(), R.data(), l);
    // TEST: The motion of point at high latitudes is sometimes not that
    //       consistent. So some wild small spurious edges will be generated.
    //       Here we filte them out!
    if (edge1->getEndPoint(FirstPoint)->getLocation().inDangerousZone ||
        edge1->getEndPoint(SecondPoint)->getLocation().inDangerousZone ||
        edge2->getEndPoint(FirstPoint)->getLocation().inDangerousZone ||
        edge2->getEndPoint(SecondPoint)->getLocation().inDangerousZone) {
        r += 2.0;
    }
    a *= r;
}