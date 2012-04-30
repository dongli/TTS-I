#include "DelaunayTriangle.hpp"
#include "Sphere.hpp"
#include "DelaunayVertex.hpp"
#include "Constants.hpp"
#include <cmath>
#include <iomanip>

using std::setw;

DelaunayTriangle::DelaunayTriangle()
{
    tip.DT = this;
    reinit();
}

DelaunayTriangle::~DelaunayTriangle()
{
}

void DelaunayTriangle::reinit()
{
    for (int i = 0; i < 3; ++i) {
        DVT[i] = NULL;
        adjDT[i] = NULL;
        subDT[i] = NULL;
    }
    tip.points->recycle();
}

void DelaunayTriangle::calcCircumcenter()
{
    double E2[3], E3[3], N[3];
    double L;
    static double eps = 1.0e-16; // Need tune.
    
    E2[0] = DVT[1]->point->getCoordinate().getX()-
            DVT[0]->point->getCoordinate().getX();
    E2[1] = DVT[1]->point->getCoordinate().getY()-
            DVT[0]->point->getCoordinate().getY();
    E2[2] = DVT[1]->point->getCoordinate().getZ()-
            DVT[0]->point->getCoordinate().getZ();
    E3[0] = DVT[2]->point->getCoordinate().getX()-
            DVT[0]->point->getCoordinate().getX();
    E3[1] = DVT[2]->point->getCoordinate().getY()-
            DVT[0]->point->getCoordinate().getY();
    E3[2] = DVT[2]->point->getCoordinate().getZ()-
            DVT[0]->point->getCoordinate().getZ();
    N[0] = E2[1]*E3[2]-E2[2]*E3[1];
    N[1] = E2[2]*E3[0]-E2[0]*E3[2];
    N[2] = E2[0]*E3[1]-E2[1]*E3[0];
    L = sqrt(N[0]*N[0]+N[1]*N[1]+N[2]*N[2]);
    if (L < eps) {
        REPORT_ERROR("Encounter collinear vertices!")
    }

    double x[3];
    x[0] = N[0]/L;
    x[1] = N[1]/L;
    x[2] = N[2]/L;
    double lon = atan2(x[1], x[0]);
    double lat = asin(x[2]);
    if (lon < 0.0) lon += PI2;
    circumcenter.setCoordinate(lon, lat);
}

void DelaunayTriangle::dump()
{
    cout << setw(10) << getID() << ":" << endl;
    for (int i = 0; i < 3; ++i) {
        cout << setw(10) << DVT[i]->getID() << endl;
    }
}
