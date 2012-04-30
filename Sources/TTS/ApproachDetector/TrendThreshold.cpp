#include "TrendThreshold.hpp"
#include "ThresholdFunction.hpp"
#include "ConfigTools.hpp"
#include "Sphere.hpp"

using namespace ApproachDetector;

Array<double, 1> TrendThreshold::P;
Array<double, 1> TrendThreshold::D;

void TrendThreshold::init()
{
    int numDistanceSection;
    ConfigTools::read("num_distance_section", numDistanceSection);
    D.resize(numDistanceSection+1);
    P.resize(D.size());
    ConfigTools::read("distance", D.size(), D.data());
    ConfigTools::read("trend_threshold", P.size(), P.data());
    D *= Sphere::radius/Rad2Deg;
}

void TrendThreshold::calc(double distance, double &percent)
{
    percent = ThresholdFunction::piecewiseCubic(P.size(), D.data(), P.data(), distance);
}