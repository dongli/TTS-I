#include "MeshAdaptor.h"
#include "PolygonManager.h"
#include "MeshManager.h"
#include "ReportMacros.h"
#include "Sphere.h"

MeshAdaptor::MeshAdaptor()
{
    REPORT_ONLINE("MeshAdaptor")
}

MeshAdaptor::~MeshAdaptor()
{
    REPORT_OFFLINE("MeshAdaptor")
}

void MeshAdaptor::init(const MeshManager &meshManager)
{
    const PointCounter &pointCounter = meshManager.pointCounter;

    overlapAreas.resize(pointCounter.points.shape());
    for (int i = 0; i < overlapAreas.extent(0); ++i)
        for (int j = 0; j < overlapAreas.extent(1); ++j)
            for (int k = 0; k < overlapAreas.extent(2); ++k)
                overlapAreas(i, j, k).resize(10);
}

#define DEBUG_DUMP \
{ \
cout << "x1: " << setw(20) << x1.getLon()*Rad2Deg << setw(20) << x1.getLat()*Rad2Deg << endl; \
cout << "x2: " << setw(20) << x2.getLon()*Rad2Deg << setw(20) << x2.getLat()*Rad2Deg << endl; \
cout << "x3: " << setw(20) << x3.getLon()*Rad2Deg << setw(20) << x3.getLat()*Rad2Deg << endl; \
cout << "x4: " << setw(20) << x4.getLon()*Rad2Deg << setw(20) << x4.getLat()*Rad2Deg << endl; \
}

void MeshAdaptor::adapt(const PolygonManager &polygonManager,
                        const MeshManager &meshManager)
{
    NOTICE("MeshAdaptor::adapt", "running ...");
    const PointCounter &pointCounter = meshManager.pointCounter;
    
    int numLon = pointCounter.counters.extent(0);
    int numLat = pointCounter.counters.extent(1);
    
    Polygon *polygon = polygonManager.polygons.front();
    for (int m = 0; m < polygonManager.polygons.size(); ++m) {
        EdgePointer *edgePointer = polygon->edgePointers.front();
        for (int n = 0; n < polygon->edgePointers.size(); ++n) {
            Vertex *vertex1 = edgePointer->getEndPoint(FirstPoint);
            Vertex *vertex2 = edgePointer->getEndPoint(SecondPoint);
            const Coordinate &x1 = vertex1->getCoordinate();
            const Coordinate &x2 = vertex2->getCoordinate();
            int I1 = vertex1->getLocation().i[4];
            int J1 = vertex1->getLocation().j[4];
            int I2 = vertex2->getLocation().i[4];
            int J2 = vertex2->getLocation().j[4];
            // 顺藤摸瓜
            int I = I1, J = J1;
            int from = -1;
            while (true) {
                // 检查是否找到了西瓜
                if (I == I2 && J == J2)
                    break;
                Coordinate x3, x4;
                Vector tmp1, tmp2;
                // 有四个方向可以摸
                // 西边界
                if (from != 2) {
                    Sphere::calcIntersectLat(x1, x2, pointCounter.lonBnds(I), x3, x4);
                    //DEBUG_DUMP;
                    if (x3.getLat() >= pointCounter.latBnds(J+1) &&
                        x3.getLat() < pointCounter.latBnds(J)) {
                        if (dot(x1.getCAR(), x3.getCAR()) > 0.0) {
                            tmp1 = cross(x1.getCAR(), x3.getCAR());
                            tmp2 = cross(x3.getCAR(), x2.getCAR());
                            if (dot(tmp1, tmp2) > 0.0) {
                                I = I-1; if (I == -1) I = numLon;
                                from = 1; continue;
                            }
                        }
                    }
                    if (x4.getLat() >= pointCounter.latBnds(J+1) &&
                        x4.getLat() < pointCounter.latBnds(J)) {
                        if (dot(x1.getCAR(), x4.getCAR()) > 0.0) {
                            tmp1 = cross(x1.getCAR(), x4.getCAR());
                            tmp2 = cross(x4.getCAR(), x2.getCAR());
                            if (dot(tmp1, tmp2) > 0.0) {
                                I = I-1; if (I == -1) I = numLon;
                                from = 1; continue;
                            }
                        }
                    }
                }
                // 东边界
                if (from != 1) {
                    Sphere::calcIntersectLat(x1, x2, pointCounter.lonBnds(I+1), x3, x4);
                    //DEBUG_DUMP;
                    if (x3.getLat() >= pointCounter.latBnds(J+1) &&
                        x3.getLat() < pointCounter.latBnds(J)) {
                        if (dot(x1.getCAR(), x3.getCAR()) > 0.0) {
                            tmp1 = cross(x1.getCAR(), x3.getCAR());
                            tmp2 = cross(x3.getCAR(), x2.getCAR());
                            if (dot(tmp1, tmp2) > 0.0) {
                                I = I+1; if (I == numLon) I = 0;
                                from = 2; continue;
                            }
                        }
                    }
                    if (x4.getLat() >= pointCounter.latBnds(J+1) &&
                        x4.getLat() < pointCounter.latBnds(J)) {
                        if (dot(x1.getCAR(), x4.getCAR()) > 0.0) {
                            tmp1 = cross(x1.getCAR(), x4.getCAR());
                            tmp2 = cross(x4.getCAR(), x2.getCAR());
                            if (dot(tmp1, tmp2) > 0.0) {
                                I = I+1; if (I == numLon) I = 0;
                                from = 2; continue;
                            }
                        }
                    }
                }
                // 北边界
                if (from != 4 && J > 0) {
                    Sphere::calcIntersectLon(x1, x2, pointCounter.latBnds(J), x3, x4);
                    //DEBUG_DUMP;
                    if (Sphere::is_lon_between(pointCounter.lonBnds(I),
                                               pointCounter.lonBnds(I+1),
                                               x3.getLon())) {
                        if (dot(x1.getCAR(), x3.getCAR()) > 0.0) {
                            tmp1 = cross(x1.getCAR(), x3.getCAR());
                            tmp2 = cross(x3.getCAR(), x2.getCAR());
                            if (dot(tmp1, tmp2) > 0.0) {
                                J = J-1;
                                from = 3; continue;
                            }
                        }
                    }
                    if (Sphere::is_lon_between(pointCounter.lonBnds(I),
                                               pointCounter.lonBnds(I+1),
                                               x4.getLon())) {
                        if (dot(x1.getCAR(), x4.getCAR()) > 0.0) {
                            tmp1 = cross(x1.getCAR(), x4.getCAR());
                            tmp2 = cross(x4.getCAR(), x2.getCAR());
                            if (dot(tmp1, tmp2) > 0.0) {
                                J = J-1;
                                from = 3; continue;
                            }
                        }
                    }
                }
                // 南边界
                if (from != 3 && J < numLat) {
                    Sphere::calcIntersectLon(x1, x2, pointCounter.latBnds(J+1), x3, x4);
                    //DEBUG_DUMP;
                    if (Sphere::is_lon_between(pointCounter.lonBnds(I),
                                               pointCounter.lonBnds(I+1),
                                               x3.getLon())) {
                        if (dot(x1.getCAR(), x3.getCAR()) > 0.0) {
                            tmp1 = cross(x1.getCAR(), x3.getCAR());
                            tmp2 = cross(x3.getCAR(), x2.getCAR());
                            if (dot(tmp1, tmp2) > 0.0) {
                                J = J+1;
                                from = 4; continue;
                            }
                        }
                    }
                    if (Sphere::is_lon_between(pointCounter.lonBnds(I),
                                               pointCounter.lonBnds(I+1),
                                               x4.getLon())) {
                        if (dot(x1.getCAR(), x4.getCAR()) > 0.0) {
                            tmp1 = cross(x1.getCAR(), x4.getCAR());
                            tmp2 = cross(x4.getCAR(), x2.getCAR());
                            if (dot(tmp1, tmp2) > 0.0) {
                                J = J+1;
                                from = 4; continue;
                            }
                        }
                    }
                }
                REPORT_ERROR("Can not find a path!");
            }
            edgePointer = edgePointer->next;
        }
        polygon = polygon->next;
    }
}