#include "Sphere.h"
#include "ReportMacros.h"
#include "Constants.h"
#include <cmath>

const double Sphere::radius = 1.0;
const double Sphere::radius2 = radius*radius;

double Sphere::calcDistance(const Coordinate &x1, const Coordinate &x2)
{
    double dlon = x1.getLon()-x2.getLon();
    double tmp1 = sin(x1.getLat())*sin(x2.getLat());
    double tmp2 = cos(x1.getLat())*cos(x2.getLat())*cos(dlon);
    return radius*acos(tmp1+tmp2);
}

void Sphere::rotate(const Coordinate &xp, const Coordinate &xo, Coordinate &xr)
{
    double dlon = xo.getLon()-xp.getLon();
    double cosLatP = cos(xp.getLat());
    double sinLatP = sin(xp.getLat());
    double cosLatO = cos(xo.getLat());
    double sinLatO = sin(xo.getLat());
    double cosDlon = cos(dlon);
    double sinDlon = sin(dlon);

    double tmp1, tmp2, tmp3;

    tmp1 = cosLatO*sinDlon;
    tmp2 = cosLatO*sinLatP*cosDlon-cosLatP*sinLatO;
    double lon = atan2(tmp1, tmp2);
    if (lon < 0.0) lon = PI2+lon;

    tmp1 = sinLatO*sinLatP;
    tmp2 = cosLatO*cosLatP*cosDlon;
    tmp3 = tmp1+tmp2;
#ifdef DEBUG
    if (tmp3 < -1.0 || tmp3 > 1.0) {
        REPORT_ERROR("tmp3 is out of range [-1, 1]!")
    }
#endif
    double lat = asin(tmp3);

    xr.set(lon, lat, xo.getLev());
}

void Sphere::inverseRotate(const Coordinate &xp, Coordinate &xo,
                           const Coordinate &xr)
{
    double cosLatP = cos(xp.getLat());
    double sinLatP = sin(xp.getLat());
    double cosLonR = cos(xr.getLon());
    double sinLonR = sin(xr.getLon());
    double cosLatR = cos(xr.getLat());
    double sinLatR = sin(xr.getLat());

    static const double eps = 1.0e-15;

    double tmp1, tmp2, tmp3;

    tmp1 = cosLatR*sinLonR;
    tmp2 = sinLatR*cosLatP+cosLatR*cosLonR*sinLatP;
#ifdef DEBUG
    if (fabs(tmp2) < eps) {
        //REPORT_WARNING("tmp2 is near zero!")
        tmp2 = 0.0;
    }
#endif
    double lon = xp.getLon()+atan2(tmp1, tmp2);
    if (lon > PI2) lon -= PI2;
    if (lon < 0.0) lon += PI2;

    tmp1 = sinLatR*sinLatP;
    tmp2 = cosLatR*cosLatP*cosLonR;
    tmp3 = tmp1-tmp2;
#ifdef DEBUG
    if (tmp3 < -1.0 || tmp3 > 1.0) {
        REPORT_ERROR("tmp3 is out of range [-1,1]!")
    }
#endif
    double lat = asin(tmp3);
    xo.set(lon, lat, xr.getLev());
}

OrientStatus Sphere::orient(const Coordinate &X1, const Coordinate &X2,
                            const Coordinate &X3)
{
    double det;
    static double eps = 1.0e-16;

    det = X3.getX()*(X1.getY()*X2.getZ()-X1.getZ()*X2.getY())-
          X3.getY()*(X1.getX()*X2.getZ()-X1.getZ()*X2.getX())+
          X3.getZ()*(X1.getX()*X2.getY()-X1.getY()*X2.getX());
    
    if (det > eps) {
        return OrientLeft;
    } else if (-det > eps) {
        return OrientRight;
    } else {
        return OrientOn;
    }
}

OrientStatus Sphere::orient(Point *endPoint1, Point *endPoint2, Point *point)
{
    double det;
    static double eps = 1.0e-16;

    double x1 = endPoint1->getCoordinate().getX();
    double x2 = endPoint2->getCoordinate().getX();
    double x3 = point->getCoordinate().getX();
    double y1 = endPoint1->getCoordinate().getY();
    double y2 = endPoint2->getCoordinate().getY();
    double y3 = point->getCoordinate().getY();
    double z1 = endPoint1->getCoordinate().getZ();
    double z2 = endPoint2->getCoordinate().getZ();
    double z3 = point->getCoordinate().getZ();

    det = x3*(y1*z2-z1*y2)-y3*(x1*z2-z1*x2)+z3*(x1*y2-y1*x2);

    if (det > eps) {
        return OrientLeft;
    } else if (-det > eps) {
        return OrientRight;
    } else {
        return OrientOn;
    }
}

bool Sphere::overlapTest(Point *point1, Point *point2)
{
    static double eps = 1.0e-4;
    double dlon, dlat;
    bool res = false;

    dlon = fabs(point1->getCoordinate().getLon()-
                point2->getCoordinate().getLon());
    dlat = fabs(point1->getCoordinate().getLat()-
                point2->getCoordinate().getLat());
    if (dlat < eps) {
        if (PI05-fabs(point1->getCoordinate().getLat()) < eps) {
            res = true;
        } else {
            if (dlon < eps || PI2-dlon < eps)
                res = true;
        }
    }
    return res;
}

InCircleStatus Sphere::inCircle(Point *point1, Point *point2,
                                Point *point3, Point *point)
{
    Point *points[3];
    points[0] = point1;
    points[1] = point2;
    points[2] = point3;
    
    double dx[3], dy[3], dz[3];
    
    for (int i = 0; i < 3; ++i) {
        dx[i] = points[i]->getCoordinate().getX()-
                point->getCoordinate().getX();
        dy[i] = points[i]->getCoordinate().getY()-
                point->getCoordinate().getY();
        dz[i] = points[i]->getCoordinate().getZ()-
                point->getCoordinate().getZ();
    }
    
    double det = dx[2]*(dy[1]*dz[0]-dy[0]*dz[1])-
                 dy[2]*(dx[1]*dz[0]-dx[0]*dz[1])+
                 dz[2]*(dx[1]*dy[0]-dx[0]*dy[1]);

    static double eps = 1.0e-16; // Tune it!
    
    if (det > eps) {
        return InsideCircle;
    } else if (-det > eps) {
        return OutsideCircle;
    } else {
        return OnCircle;
    }
}

int Sphere::inTriangle(Point *vertex1, Point *vertex2,
                       Point *vertex3, Point *point)
{
    Point *vertices[3];
    vertices[0] = vertex1;
    vertices[1] = vertex2;
    vertices[2] = vertex3;

    for (int i = 0; i < 3; ++i)
        if (Sphere::overlapTest(vertices[i], point))
            return -(i+1);

    int ip1, im1, k = -1, onPlane[2], res;
    OrientStatus ret;
    
    for (int i = 0; i < 3; ++i) {
        ip1 = i != 2 ? i+1 : 0;
        ret = Sphere::orient(vertices[i], vertices[ip1], point);
        if (ret == OrientRight) { // outside
            return -4;
        } else if (ret == OrientOn) {
            im1 = i != 0 ? i-1 : 2;
            onPlane[++k] = im1;
        }
    }
    if (k == -1) { // inside
        res = 4;
    } else if (k == 0) { // on the edge
        res = onPlane[k]+1;
    } else if (k == 1) { // on the vertex
        if (onPlane[0] == 0 && onPlane[1] == 1) {
            res = -3;
        } else if (onPlane[0] == 1 && onPlane[1] == 0) {
            res = -3;
        } else if (onPlane[0] == 1 && onPlane[1] == 2) {
            res = -1;
        } else if (onPlane[0] == 2 && onPlane[1] == 1) {
            res = -1;
        } else if (onPlane[0] == 0 && onPlane[1] == 2) {
            res = -2;
        } else if (onPlane[0] == 2 && onPlane[1] == 0) {
            res = -2;
        }
    }
    return res;

}