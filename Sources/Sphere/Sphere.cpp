#include "Sphere.h"
#include "ReportMacros.h"
#include "Constants.h"
#include "Polygon.h"
#include <cmath>
#include "mpreal.h"

using namespace mpfr;

double Sphere::radius = 1.0;
double Sphere::radius2 = radius*radius;

void Sphere::setRadius(double r)
{
    radius = r;
    radius2 = r*r;
}

double Sphere::calcDistance(const Coordinate &x1, const Coordinate &x2)
{
    double dlon = x1.getLon()-x2.getLon();
    double tmp1 = sin(x1.getLat())*sin(x2.getLat());
    double tmp2 = cos(x1.getLat())*cos(x2.getLat())*cos(dlon);
    double tmp3 = fmin(1.0, fmax(-1.0, tmp1+tmp2));
    return radius*acos(tmp3);
}

bool Sphere::project(const Coordinate &x1, const Coordinate &x2,
                     const Coordinate &x3, Coordinate &x4, double &distance)
{
    Coordinate x1r, x2r, x3r, x4r, xpr, xp;
    double lon;
    // find the rotating north pole that make arc x1->x2 equator
    rotate(x1, x2, x2r);
    xpr.setSPH(x2r.getLon()-PI05, 0.0);
    inverseRotate(x1, xp, xpr);
    // rotate according to rotating north pole
    rotate(xp, x1, x1r);
    rotate(xp, x2, x2r);
    rotate(xp, x3, x3r);
    if (fabs(x3r.getLat()-PI05) < EPS) {
        REPORT_ERROR("Ambiguous point!")
    }
    if (x1r.getLon() < x2r.getLon()) {
        if (x3r.getLon() >= x2r.getLon() || x3r.getLon() <= x1r.getLon()) {
            distance = fabs(x3r.getLat())*Sphere::radius;
            x4r.setSPH(x3r.getLon(), 0.0);
            inverseRotate(xp, x4, x4r);
            return true;
        }
        lon = x3r.getLon()-PI;
        if (lon < 0.0)
            lon += PI2;
        if (lon >= x2r.getLon() || lon <= x1r.getLon()) {
            distance = (PI-fabs(x3r.getLat()))*Sphere::radius;
            x4r.setSPH(lon, 0.0);
            inverseRotate(xp, x4, x4r);
            return true;
        }
    } else {
        if (x3r.getLon() >= x2r.getLon() && x3r.getLon() <= x1r.getLon()) {
            distance = fabs(x3r.getLat())*Sphere::radius;
            x4r.setSPH(x3r.getLon(), 0.0);
            inverseRotate(xp, x4, x4r);
            return true;
        }
        lon = x3r.getLon()-PI;
        if (lon < 0.0)
            lon += PI2;
        if (lon >= x2r.getLon() && lon <= x1r.getLon()) {
            distance = (PI-fabs(x3r.getLat()))*Sphere::radius;
            x4r.setSPH(lon, 0.0);
            inverseRotate(xp, x4, x4r);
            return true;
        }
    }
    distance = UndefinedDistance;
    return false;
}

inline bool Sphere::isProject(const Coordinate &x1, const Coordinate &x2,
                       const Coordinate &x3)
{
    Vector tmp1, tmp2;
    tmp1 = cross(x1.getCAR(), x3.getCAR());
    tmp2 = cross(x3.getCAR(), x2.getCAR());
    if (dot(tmp1, tmp2) > 0.0)
        return true;
    else
        return false;
}

bool Sphere::isProject(Point *point1, Point *point2, Point *point3)
{
    return isProject(point1->getCoordinate(), point2->getCoordinate(),
                     point3->getCoordinate());
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
        if (fabs(tmp3)-1.0 < EPS) {
            REPORT_WARNING("tmp3 is out of range [-1, 1]!");
        } else
            REPORT_ERROR("tmp3 is out of range [-1, 1]!");
    }
#endif
    tmp3 = fmin(1.0, fmax(-1.0, tmp3));
    double lat = asin(tmp3);

    xr.setSPH(lon, lat, xo.getLev());
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

    double tmp1, tmp2, tmp3;

    tmp1 = cosLatR*sinLonR;
    tmp2 = sinLatR*cosLatP+cosLatR*cosLonR*sinLatP;
#ifdef DEBUG
    static const double eps = 1.0e-15;
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
    if (tmp3 < -1.0 || tmp3 > 1.0)
        REPORT_ERROR("tmp3 is out of range [-1,1]!");
#endif
    tmp3 = fmin(1.0, fmax(-1.0, tmp3));
    double lat = asin(tmp3);
    xo.setSPH(lon, lat, xr.getLev());
}

void Sphere::calcMiddlePoint(const Coordinate &x1, const Coordinate &x2,
                             Coordinate &x)
{
    Coordinate xr;
    Sphere::rotate(x1, x2, xr);
    double dlat = (PI05-xr.getLat())*0.5;
    xr.setSPH(xr.getLon(), PI05-dlat);
    Sphere::inverseRotate(x1, x, xr);
}

void Sphere::calcCentroid(Polygon const *polygon, Coordinate &x)
{
    double X = 0.0, Y = 0.0, Z = 0.0;
    EdgePointer *edgePointer = polygon->edgePointers.front();
    for (int i = 0; i < polygon->edgePointers.size(); ++i) {
        Vertex *vertex = edgePointer->getEndPoint(FirstPoint);
        X += vertex->getCoordinate().getX();
        Y += vertex->getCoordinate().getY();
        Z += vertex->getCoordinate().getZ();
        edgePointer = edgePointer->next;
    }
    x.setCAR(X, Y, Z);
}

inline bool Sphere::isIntersect(const Coordinate &x1, const Coordinate &x2,
                                const Coordinate &x3, const Coordinate &x4)
{
    static Coordinate x[2];
    if (calcIntersect(x1, x2, x3, x4, x[0], x[1])) {
        // BUG: Clarify the validity of the judgement. Which part of the
        //      great-circle arc do we refer?
        Vector tmp1, tmp2, tmp3, tmp4;
        for (int i = 0; i < 2; ++i) {
            tmp1 = cross(x1.getCAR(), x[i].getCAR());
            tmp2 = cross(x2.getCAR(), x[i].getCAR());
            tmp3 = cross(x3.getCAR(), x[i].getCAR());
            tmp4 = cross(x4.getCAR(), x[i].getCAR());
            if (dot(tmp1, tmp2) < 0.0 && dot(tmp3, tmp4) < 0.0)
                return true;
        }
    }
    return false;
}

bool Sphere::isIntersect(Point *point1, Point *point2,
                         Point *point3, Point *point4)
{
    return isIntersect(point1->getCoordinate(), point2->getCoordinate(),
                       point3->getCoordinate(), point4->getCoordinate());
}

bool Sphere::calcIntersect(const Coordinate &x1, const Coordinate &x2,
                           const Coordinate &x3, const Coordinate &x4,
                           Coordinate &x5, Coordinate &x6)
{
    static const double eps = 1.0e-12;
    Vector n1 = cross(x1.getCAR(), x2.getCAR());
    Vector n2 = cross(x3.getCAR(), x4.getCAR());
    Vector v = cross(n1, n2);

    double r = norm(v);

    if (r > eps) {
        v /= r;
    } else {
        return false;
    }

    double lat1 = asin(v[2]);
    double lat2 = -lat1;
    double lon1 = atan2(v[1], v[0]);
    double lon2 = lon1-PI;

    if (lon1 < 0.0) lon1 += PI2;
    if (lon1 > PI2) lon1 -= PI2;
    if (lon2 < 0.0) lon2 += PI2;
    if (lon2 > PI2) lon2 -= PI2;

    x5.setSPH(lon1, lat1);
    x6.setSPH(lon2, lat2);

    return true;
}

bool Sphere::calcIntersectLat(const Coordinate &x1, const Coordinate &x2,
                              double lon, double lat1, double lat2,
                              Coordinate &x)
{
    Coordinate x3, x4, X[2];
    x3.setSPH(lon, 0.0);
    x4.setSPH(lon, PI05);
    if (calcIntersect(x1, x2, x3, x4, X[0], X[1])) {
        Vector tmp1, tmp2;
        for (int i = 0; i < 2; ++i) {
            if (Sphere::is_lon_eq(X[i].getLon(), lon) &&
                (X[i].getLat() >= lat2 && X[i].getLat() < lat1))
                if (dot(x1.getCAR(), X[i].getCAR()) > 0.0) {
                    tmp1 = cross(x1.getCAR(), X[i].getCAR());
                    tmp2 = cross(x2.getCAR(), X[i].getCAR());
                    if (dot(tmp1, tmp2) < 0.0) {
                        x = X[i];
                        return true;
                    }
                }
        }
    }
    return false;
}

bool Sphere::calcIntersectLon(const Coordinate &x1, const Coordinate &x2,
                              double lon1, double lon2, double lat,
                              Coordinate &x, bool useMPFR)
{
    static const double eps = 1.0e-12;
    if (useMPFR) {
        mpreal::set_default_prec(128);
        mpreal a = x1.getY()*x2.getZ()-x1.getZ()*x2.getY();
        mpreal b = x1.getZ()*x2.getX()-x1.getX()*x2.getZ();
        mpreal c = x1.getX()*x2.getY()-x1.getY()*x2.getX();

        mpreal z = sin(lat);
        mpreal z2 = z*z;
        mpreal a2 = a*a;
        mpreal a2_plus_b2 = a2+b*b;
        mpreal d = b*c*z/a2_plus_b2;
        mpreal e2 = d*d-((z2-1.0)*a2+z2*c*c)/a2_plus_b2;
        if (e2 < 0.0)
            return false;
        mpreal e = sqrt(e2);

        mpreal y1 = -d+e;
        mpreal y2 = -d-e;

        mpreal lon[2];
        if (fabs(a) > eps) {
            lon[0] = atan2(y1, (-b*y1-c*z)/a);
            lon[1] = atan2(y2, (-b*y2-c*z)/a);
        } else {
            lon[0] = atan2(0.0, -b*y1-c*z);
            lon[1] = atan2(0.0, -b*y2-c*z);
        }
        if (lon[0] < 0.0) lon[0] += PI2;
        if (lon[0] > PI2) lon[0] -= PI2;
        if (lon[1] < 0.0) lon[1] += PI2;
        if (lon[1] > PI2) lon[1] -= PI2;

        Coordinate X[2];

        X[0].setSPH(lon[0].toDouble(), lat);
        X[1].setSPH(lon[1].toDouble(), lat);

        Vector tmp1, tmp2;
        for (int i = 0; i < 2; ++i) {
            if (fabs(X[i].getLat()-lat) < EPS &&
                is_lon_between(lon1, lon2, lon[i].toDouble()))
                if (dot(x1.getCAR(), X[i].getCAR()) > 0.0) {
                    tmp1 = cross(x1.getCAR(), X[i].getCAR());
                    tmp2 = cross(x2.getCAR(), X[i].getCAR());
                    if (dot(tmp1, tmp2) < 0.0) {
                        x = X[i];
                        return true;
                    }
                }
        }
        return false;
    } else {
        double a = x1.getY()*x2.getZ()-x1.getZ()*x2.getY();
        double b = x1.getZ()*x2.getX()-x1.getX()*x2.getZ();
        double c = x1.getX()*x2.getY()-x1.getY()*x2.getX();

        double z = sin(lat);
        double z2 = z*z;
        double a2 = a*a;
        double a2_plus_b2 = a2+b*b;
        double d = b*c*z/a2_plus_b2;
        double e2 = d*d-((z2-1.0)*a2+z2*c*c)/a2_plus_b2;
        if (e2 < 0.0)
            return false;
        double e = std::sqrt(e2);
        
        double y1 = -d+e;
        double y2 = -d-e;

        double lon[2];
        if (fabs(a) > eps) {
//#ifdef DEBUG
//        if (fabs(pow(cos(lat), 2)-pow(y1, 2)-pow((-b*y1-c*z)/a, 2)) > 1.0e-10) {
//            cout << fabs(pow(cos(lat), 2)-pow(y1, 2)-pow((-b*y1-c*z)/a, 2)) << endl;
//            REPORT_WARNING("Unconsistent result!");
//        }
//#endif
            lon[0] = atan2(y1, (-b*y1-c*z)/a);
            lon[1] = atan2(y2, (-b*y2-c*z)/a);
        } else {
            lon[0] = atan2(0.0, -b*y1-c*z);
            lon[1] = atan2(0.0, -b*y2-c*z);
        }
        if (lon[0] < 0.0) lon[0] += PI2;
        if (lon[0] > PI2) lon[0] -= PI2;
        if (lon[1] < 0.0) lon[1] += PI2;
        if (lon[1] > PI2) lon[1] -= PI2;

        Coordinate X[2];

        X[0].setSPH(lon[0], lat);
        X[1].setSPH(lon[1], lat);

        Vector tmp1, tmp2;
        for (int i = 0; i < 2; ++i) {
            if (fabs(X[i].getLat()-lat) < EPS && is_lon_between(lon1, lon2, lon[i]))
                if (dot(x1.getCAR(), X[i].getCAR()) > 0.0) {
                    tmp1 = cross(x1.getCAR(), X[i].getCAR());
                    tmp2 = cross(x2.getCAR(), X[i].getCAR());
                    if (dot(tmp1, tmp2) < 0.0) {
                        x = X[i];
                        return true;
                    }
                }
        }
        return false;
    }
}

OrientStatus Sphere::orient(const Coordinate &x1, const Coordinate &x2,
                            const Coordinate &x3)
{
    static const double eps = 1.0e-16;

    double det = x3.getX()*(x1.getY()*x2.getZ()-x1.getZ()*x2.getY())-
                 x3.getY()*(x1.getX()*x2.getZ()-x1.getZ()*x2.getX())+
                 x3.getZ()*(x1.getX()*x2.getY()-x1.getY()*x2.getX());

    if (det > eps) {
        return OrientLeft;
    } else if (-det > eps) {
        return OrientRight;
    } else {
        return OrientOn;
    }
}

bool Sphere::is_lon_between(double lon1, double lon2, double lon)
{
    if (lon1 < 0) lon1 += PI2;
    if (lon1 > PI2) lon1 -= PI2;
    if (lon2 < 0) lon2 += PI2;
    if (lon2 > PI2) lon2 -= PI2;
    if (lon1 < lon2) {
        if (lon >= lon1 && lon < lon2)
            return true;
        else
            return false;
    } else {
        if ((lon >= lon1 && lon < PI2) || (lon >= 0.0 && lon < lon2))
            return true;
        else
            return false;
    }
}

bool Sphere::is_lon_gt(double lon1, double lon2)
{
    double dlon = lon1-lon2;
    if (dlon > 0.0) {
        if (PI2-dlon > dlon)
            return true;
        else
            return false;
    } else
        if (PI2+dlon < -dlon)
            return true;
        else
            return false;
}

bool Sphere::is_lon_lt(double lon1, double lon2)
{
    double dlon = lon1-lon2;
    if (dlon > 0.0) {
        if (PI2-dlon > dlon)
            return false;
        else
            return true;
    } else
        if (PI2+dlon < -dlon)
            return false;
        else
            return true;
}

bool Sphere::is_lon_eq(double lon1, double lon2)
{
    if (fabs(lon1-lon2) < EPS)
        return true;
    else if (fabs(PI2-fabs(lon1-lon2)) < EPS)
        return true;
    return false;
}

double Sphere::diff_lon(double lon1, double lon2)
{
    double dlon;
    if (lon1 > lon2) lon1 -= PI2;
    dlon = lon2-lon1;
    return dlon;
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
