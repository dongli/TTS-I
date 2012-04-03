#include "MeshAdaptor.h"
#include "ReportMacros.h"
#include "Sphere.h"
#include "CoverMask.h"

#include <map>
#include <list>

using std::map;
using std::list;

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
    overlapAreaList.resize(pointCounter.points.shape());
}

inline double MeshAdaptor::calcCorrectArea(const Coordinate &x1,
                                           const Coordinate &x2,
                                           const Vector &normVector,
                                           int signFlag)
{
#ifdef DEBUG
    assert(x1.getLat() == x2.getLat());
#endif
    // -------------------------------------------------------------------------
    // judge in which hemisphere
    // Note: Input "signFlag" is based on the assumption that the area is on the
    //       northern hemisphere.
    if (x1.getLat() == 0.0) return 0.0;
    bool isInNorthHemisphere = x1.getLat() >= 0.0 ? true : false;
    if (x1.getLat() < 0.0) {
        signFlag = -signFlag;
    }
    // -------------------------------------------------------------------------
    // set two projection on the equator
    Coordinate x3, x4;
    x3.setSPH(x1.getLon(), 0.0);
    x4.setSPH(x2.getLon(), 0.0);
    // -------------------------------------------------------------------------
    // calculate two normal vectors of the longitudinal lines
    Vector normVector1, normVector2;
    if (isInNorthHemisphere) {
        normVector1 = norm_cross(x3.getCAR(), x1.getCAR());
        normVector2 = norm_cross(x2.getCAR(), x4.getCAR());
    } else {
        normVector1 = norm_cross(x1.getCAR(), x3.getCAR());
        normVector2 = norm_cross(x4.getCAR(), x2.getCAR());
    }
    // -------------------------------------------------------------------------
    // calculate two angles between the longitudinal lines and the assumed
    // great circel arc x2->x1 which should be latitudinal line
    double angle1, angle2;
    if (isInNorthHemisphere)
        switch (signFlag) {
            case 1:
                angle1 = Sphere::calcAngle(normVector, normVector1, x1);
                angle2 = Sphere::calcAngle(normVector2, normVector, x2);
                break;
            case -1:
                angle1 = Sphere::calcAngle(-normVector, normVector1, x1);
                angle2 = Sphere::calcAngle(normVector2, -normVector, x2);
                break;
            default:
                REPORT_ERROR("Unknown sign flag!");
        }
    else
        switch (signFlag) {
            case 1:
                angle1 = Sphere::calcAngle(normVector1, normVector, x1);
                angle2 = Sphere::calcAngle(normVector, normVector2, x2);
                break;
            case -1:
                angle1 = Sphere::calcAngle(normVector1, -normVector, x1);
                angle2 = Sphere::calcAngle(-normVector, normVector2, x2);
                break;
            default:
                REPORT_ERROR("Unknown sign flag!");
        }
    // -------------------------------------------------------------------------
    // calculate two areas with all great circle arc edges and real latitudinal
    // line respectively
    double area1, area2;
    area1 = (angle1+angle2-PI)*Sphere::radius2;
    double dlon = x2.getLon()-x1.getLon();
    if (x1.getLon() < 0.0) {
        if (x2.getLon() > PI2+x1.getLon())
            dlon -= PI2;
    } else if (x1.getLon() > x2.getLon()) {
        dlon += PI2;
    }
    area2 = Sphere::radius2*dlon*fabs(sin(x1.getLat()));
#ifdef DEBUG
    assert(area1 >= 0.0);
    assert(area2 >= 0.0);
#endif
    // -------------------------------------------------------------------------
    // return the area difference
    if (isInNorthHemisphere)
        return area1-area2;
    else
        return area2-area1;
}

double MeshAdaptor::calcOverlapArea(int I, int J, Bnd from, Bnd to,
                                    int &bndDiff,
                                    double lonBnd1, double lonBnd2,
                                    double latBnd1, double latBnd2,
                                    Coordinate x0, EdgePointer *edgePointer0,
                                    Coordinate x1, EdgePointer *edgePointer1)
{
#ifdef DEBUG
    assert(from != NullBnd && to != NullBnd);
#endif
    // -------------------------------------------------------------------------
    Location::Pole isPole = Location::Null;
    int i, j;
    Vector normVector0 = edgePointer0->getNormVector();
    Vector normVector1 = edgePointer1->getNormVector();
    int numCellEdge, numPolygonEdge = 1, numEdge;
    static Array<Vector, 1> normVectors;
    static Array<Coordinate, 1> x;
    static Array<double, 1> polygonAngles, angles;
    double excess, area;
    // -------------------------------------------------------------------------
    // 
    bndDiff = from-to;
    if (bndDiff < -1)
        bndDiff += 4;
    else if (bndDiff > 2)
        bndDiff -= 4;
    else if (bndDiff == 0) {
        // Note: When "from" is equal with "to", we can not judge the direction
        //       of the edge from them
        switch (from) {
            case EastBnd:
                if (x0.getLat() < x1.getLat()) bndDiff = 4;
                break;
            case WestBnd:
                if (x0.getLat() > x1.getLat()) bndDiff = 4;
                break;
            case NorthBnd:
                if (Sphere::is_lon_gt(x0.getLon(), x1.getLon())) bndDiff = 4;
                break;
            case SouthBnd:
                if (Sphere::is_lon_gt(x1.getLon(), x0.getLon())) bndDiff = 4;
                break;
            default:
                REPORT_ERROR("Unknown boundary!");
        }
    }
    // -------------------------------------------------------------------------
    // mark pole cell
    if (fabs(latBnd1-PI05) < EPS)
        isPole = Location::NorthPole;
    else if (fabs(latBnd2+PI05) < EPS)
        isPole = Location::SouthPole;
    // -------------------------------------------------------------------------
    // get the number of cell edges
    switch (bndDiff) {
        case 0:
            numCellEdge = 1;
            break;
        case 1:
            numCellEdge = 2;
            break;
        case 2:
            numCellEdge = 3;
            if ((isPole == Location::NorthPole && from == WestBnd) ||
                (isPole == Location::SouthPole && from == EastBnd))
                numCellEdge = 2;
            break;
        case -1:
            if (isPole == Location::Null)
                numCellEdge = 4;
            else
                numCellEdge = 3;
            break;
        case 4:
            if (isPole == Location::Null)
                numCellEdge = 5;
            else
                numCellEdge = 4;
            break;
        default:
            REPORT_ERROR("Unknown boundary difference!");
    }
    // -------------------------------------------------------------------------
    // get the number of polygon edges
    EdgePointer *edgePointer = edgePointer0;
    while (edgePointer != edgePointer1) {
        numPolygonEdge++;
        edgePointer = edgePointer->next;
    }
    polygonAngles.resize(numPolygonEdge-1);
    edgePointer = edgePointer0->next;
    for (i = 0; i < numPolygonEdge-1; ++i) {
        polygonAngles(i) = edgePointer->getAngle(NewTimeLevel);
        edgePointer = edgePointer->next;
    }
    // -------------------------------------------------------------------------
    // get the normal vector of the cell edges
    normVectors.resize(numCellEdge);
    x.resize(numCellEdge-1);
    switch (bndDiff) {
        case 0:
            break;
        case 1:
            switch (from) {
                case EastBnd:
                    if (isPole != Location::SouthPole)
                        x(0).setSPH(lonBnd2, latBnd2);
                    break;
                case WestBnd:
                    if (isPole != Location::NorthPole)
                        x(0).setSPH(lonBnd1, latBnd1);
                    break;
                case NorthBnd:
                    if (isPole != Location::NorthPole)
                        x(0).setSPH(lonBnd2, latBnd1);
                    break;
                case SouthBnd:
                    if (isPole != Location::SouthPole)
                        x(0).setSPH(lonBnd1, latBnd2);
                    break;
                default:
                    REPORT_ERROR("Unknown boundary!");
            }
            break;
        case 2:
            switch (from) {
                case EastBnd:
                    if (isPole != Location::SouthPole) {
                        x(0).setSPH(lonBnd1, latBnd2);
                        x(1).setSPH(lonBnd2, latBnd2);
                    } else
                        x(0).setSPH(0.0, -PI05);
                    break;
                case WestBnd:
                    if (isPole != Location::NorthPole) {
                        x(0).setSPH(lonBnd2, latBnd1);
                        x(1).setSPH(lonBnd1, latBnd1);
                    } else
                        x(0).setSPH(0.0, PI05);
                    break;
                case NorthBnd:
                    x(0).setSPH(lonBnd2, latBnd2);
                    x(1).setSPH(lonBnd2, latBnd1);
                    break;
                case SouthBnd:
                    x(0).setSPH(lonBnd1, latBnd1);
                    x(1).setSPH(lonBnd1, latBnd2);
                    break;
                default:
                    REPORT_ERROR("Unknown boundary!");
            }
            break;
        case -1:
            switch (from) {
                case EastBnd:
                    if (isPole == Location::Null) {
                        x(0).setSPH(lonBnd1, latBnd1);
                        x(1).setSPH(lonBnd1, latBnd2);
                        x(2).setSPH(lonBnd2, latBnd2);
                    } else if (isPole == Location::SouthPole) {
                        x(0).setSPH(lonBnd1, latBnd1);
                        x(1).setSPH(0.0, -PI05);
                    }
                    break;
                case WestBnd:
                    if (isPole == Location::Null) {
                        x(0).setSPH(lonBnd2, latBnd2);
                        x(1).setSPH(lonBnd2, latBnd1);
                        x(2).setSPH(lonBnd1, latBnd1);
                    } else if (isPole == Location::NorthPole) {
                        x(0).setSPH(lonBnd2, latBnd2);
                        x(1).setSPH(0.0, PI05);
                    }
                    break;
                case NorthBnd:
                    if (isPole == Location::Null) {
                        x(0).setSPH(lonBnd1, latBnd2);
                        x(1).setSPH(lonBnd2, latBnd2);
                        x(2).setSPH(lonBnd2, latBnd1);
                    } else if (isPole == Location::SouthPole) {
                        x(0).setSPH(0.0, -PI05);
                        x(1).setSPH(lonBnd2, latBnd1);
                    }
                    break;
                case SouthBnd:
                    if (isPole == Location::Null) {
                        x(0).setSPH(lonBnd2, latBnd1);
                        x(1).setSPH(lonBnd1, latBnd1);
                        x(2).setSPH(lonBnd1, latBnd2);
                    } else if (isPole == Location::NorthPole) {
                        x(0).setSPH(0.0, PI05);
                        x(1).setSPH(lonBnd1, latBnd2);
                    }
                    break;
                default:
                    REPORT_ERROR("Unknown boundary!");
            }
            break;
        case 4:
            switch (from) {
                case EastBnd:
                    if (isPole == Location::Null) {
                        x(0).setSPH(lonBnd2, latBnd1);
                        x(1).setSPH(lonBnd1, latBnd1);
                        x(2).setSPH(lonBnd1, latBnd2);
                        x(3).setSPH(lonBnd2, latBnd2);
                    } else if (isPole == Location::NorthPole) {
                        x(0).setSPH(0.0, PI05);
                        x(1).setSPH(lonBnd1, latBnd2);
                        x(2).setSPH(lonBnd2, latBnd2);
                    } else if (isPole == Location::SouthPole) {
                        x(0).setSPH(lonBnd2, latBnd1);
                        x(1).setSPH(lonBnd1, latBnd1);
                        x(2).setSPH(0.0, -PI05);
                    }
                    break;
                case WestBnd:
                    if (isPole == Location::Null) {
                        x(0).setSPH(lonBnd1, latBnd2);
                        x(1).setSPH(lonBnd2, latBnd2);
                        x(2).setSPH(lonBnd2, latBnd1);
                        x(3).setSPH(lonBnd1, latBnd1);
                    } else if (isPole == Location::NorthPole) {
                        x(0).setSPH(lonBnd1, latBnd2);
                        x(1).setSPH(lonBnd2, latBnd2);
                        x(2).setSPH(0.0, PI05);
                    } else if (isPole == Location::SouthPole) {
                        x(0).setSPH(0.0, -PI05);
                        x(1).setSPH(lonBnd2, latBnd1);
                        x(2).setSPH(lonBnd1, latBnd1);
                    }
                    break;
                case NorthBnd:
                    if (isPole == Location::Null) {
                        x(0).setSPH(lonBnd1, latBnd1);
                        x(1).setSPH(lonBnd1, latBnd2);
                        x(2).setSPH(lonBnd2, latBnd2);
                        x(3).setSPH(lonBnd2, latBnd1);
                    } else if (isPole == Location::SouthPole) {
                        x(0).setSPH(lonBnd1, latBnd1);
                        x(1).setSPH(0.0, -PI05);
                        x(2).setSPH(lonBnd2, latBnd1);
                    }
                    break;
                case SouthBnd:
                    if (isPole == Location::Null) {
                        x(0).setSPH(lonBnd2, latBnd2);
                        x(1).setSPH(lonBnd2, latBnd1);
                        x(2).setSPH(lonBnd1, latBnd1);
                        x(3).setSPH(lonBnd1, latBnd2);
                    } else if (isPole == Location::NorthPole) {
                        x(0).setSPH(lonBnd2, latBnd2);
                        x(1).setSPH(0.0, PI05);
                        x(2).setSPH(lonBnd1, latBnd2);
                    }
                    break;
                default:
                    REPORT_ERROR("Unknown boundary!");
            }
            break;
        default:
            REPORT_ERROR("Unknown boundary difference!");
    }
    if (numCellEdge != 1) {
        if (x1.getLon() != x(0).getLon() || x1.getLat() != x(0).getLat()) {
            normVectors(0) = norm_cross(x(0).getCAR(), x1.getCAR());
            for (i = 1; i < numCellEdge-1; ++i)
                normVectors(i) = norm_cross(x(i).getCAR(), x(i-1).getCAR());
        } else {
            numCellEdge--;
            for (i = 0; i < numCellEdge-1; ++i)
                normVectors(i) = norm_cross(x(i+1).getCAR(), x(i).getCAR());
        }
        if (x0.getLon() != x(i-1).getLon() || x0.getLat() != x(i-1).getLat())
            normVectors(i) = norm_cross(x0.getCAR(), x(i-1).getCAR());
        else
            numCellEdge--;
    } else
        normVectors(0) = norm_cross(x0.getCAR(), x1.getCAR());
    // -------------------------------------------------------------------------
numerical_tolerance_label:
    // -------------------------------------------------------------------------
    // get or calculate the internal angles
    numEdge = numCellEdge+numPolygonEdge;
    angles.resize(numEdge);
    //
    for (i = 0; i < numPolygonEdge-1; ++i)
        angles(i) = polygonAngles(i);
    //
    angles(i++) = Sphere::calcAngle(normVector1, normVectors(0), x1);
    j = 0;
    for (; i < numEdge-1; ++i) {
        angles(i) = Sphere::calcAngle(normVectors(j), normVectors(j+1), x(j));
        j++;
    }
    angles(i) = Sphere::calcAngle(normVectors(j), normVector0, x0);
    // -------------------------------------------------------------------------
    // numerical tolerance
    static const double smallAngle = 1.0/Rad2Deg;
    if (PI2-angles(numEdge-1) < smallAngle) {
        numPolygonEdge--;
        // set the virtual intersection
        if (from == NorthBnd || from == SouthBnd)
            x0.setSPH(edgePointer0->getEndPoint(SecondPoint)
                   ->getCoordinate().getLon(), x0.getLat());
        else
            x0.setSPH(x0.getLon(), edgePointer0->getEndPoint(SecondPoint)
                   ->getCoordinate().getLat());
        // calculate the virtual normal vector of the polygon edge
        edgePointer0 = edgePointer0->next;
        if (numPolygonEdge == 1) {
            normVector0 = norm_cross(x1.getCAR(), x0.getCAR());
            normVector1 = normVector0;
        } else
            normVector0 = norm_cross(edgePointer0->getEndPoint(SecondPoint)
                                     ->getCoordinate().getCAR(), x0.getCAR());
        // calculate the virtual normal vector of the cell edge
        if (numCellEdge == 1)
            normVectors(0) = norm_cross(x0.getCAR(), x1.getCAR());
        else
            normVectors(numCellEdge-1) = norm_cross(x0.getCAR(),
                                                    x(numCellEdge-2).getCAR());
        // set the virtual polygon angles
        if (edgePointer0 != edgePointer1) {
            polygonAngles(0) = Sphere::calcAngle
            (normVector0, edgePointer0->next->getNormVector(),
             edgePointer0->getEndPoint(SecondPoint)->getCoordinate());
            for (i = 1; i < numPolygonEdge-1; ++i)
                polygonAngles(i) = polygonAngles(i+1);
        }
        goto numerical_tolerance_label;
    } else if (PI2-angles(numPolygonEdge-1) < smallAngle) {
        numPolygonEdge--;
        if (to == NorthBnd || to == SouthBnd)
            x1.setSPH(edgePointer1->getEndPoint(FirstPoint)
                   ->getCoordinate().getLon(), x1.getLat());
        else
            x1.setSPH(x1.getLon(), edgePointer1->getEndPoint(FirstPoint)
                   ->getCoordinate().getLat());
        edgePointer1 = edgePointer1->prev;
        if (numPolygonEdge == 1) {
            normVector1 = norm_cross(x1.getCAR(), x0.getCAR());
            normVector0 = normVector1;
        } else
            normVector1 = norm_cross(x1.getCAR(), edgePointer1->
                                     getEndPoint(FirstPoint)->
                                     getCoordinate().getCAR());
        if (numCellEdge == 1)
            normVectors(0) = norm_cross(x0.getCAR(), x1.getCAR());
        else
            normVectors(0) = norm_cross(x(0).getCAR(), x1.getCAR());
        if (edgePointer1 != edgePointer0) {
            polygonAngles(numPolygonEdge-2) = Sphere::calcAngle
            (edgePointer1->prev->getNormVector(), normVector1,
             edgePointer1->getEndPoint(FirstPoint)->getCoordinate());
        }
        goto numerical_tolerance_label;
    }
    // -------------------------------------------------------------------------
    // calculate the area of overlapping polygon (assuming all great circle
    // arc edges)
    excess = 0.0;
    for (i = 0; i < numEdge; ++i)
        excess += angles(i);
    excess -= (numEdge-2)*PI;
    area = excess*Sphere::radius2;
    // -------------------------------------------------------------------------
    // correct the area if some edges are latitudinal lines
    switch (bndDiff) {
        case 0:
            switch (from) {
                case EastBnd:
                    break;
                case WestBnd:
                    break;
                case NorthBnd:
                    area -= calcCorrectArea(x0, x1, normVectors(0), 1);
                    break;
                case SouthBnd:
                    area += calcCorrectArea(x1, x0, normVectors(0), -1);
                    break;
                default:
                    REPORT_ERROR("Unknown boundary!");
            }
            break;
        case 1:
            switch (from) {
                case EastBnd:
                    if (isPole != Location::SouthPole)
                        area += calcCorrectArea(x1, x(0), normVectors(0), -1);
                    break;
                case WestBnd:
                    if (isPole != Location::NorthPole)
                        area -= calcCorrectArea(x(0), x1, normVectors(0), 1);
                    break;
                case NorthBnd:
                    if (isPole != Location::NorthPole)
                        area -= calcCorrectArea(x0, x(0), normVectors(1), 1);
                    break;
                case SouthBnd:
                    if (isPole != Location::SouthPole)
                        area += calcCorrectArea(x(0), x0, normVectors(1), -1);
                    break;
                default:
                    REPORT_ERROR("Unknown boundary!");
            }
            break;
        case 2:
            switch (from) {
                case EastBnd:
                    if (isPole != Location::SouthPole)
                        area += calcCorrectArea(x(0), x(1), normVectors(1), -1);
                    break;
                case WestBnd:
                    if (isPole != Location::NorthPole)
                        area -= calcCorrectArea(x(1), x(0), normVectors(1), 1);
                    break;
                case NorthBnd:
                    area -= calcCorrectArea(x0, x(1), normVectors(2), 1);
                    area += calcCorrectArea(x1, x(0), normVectors(0), -1);
                    break;
                case SouthBnd:
                    area -= calcCorrectArea(x(0), x1, normVectors(0), 1);
                    area += calcCorrectArea(x(1), x0, normVectors(2), -1);
                    break;
                default:
                    REPORT_ERROR("Unknown boundary!");
            }
            break;
        case -1:
            switch (from) {
                case EastBnd:
                    area -= calcCorrectArea(x(0), x1, normVectors(0), 1);
                    if (isPole == Location::Null)
                        area += calcCorrectArea(x(1), x(2), normVectors(2), -1);
                    break;
                case WestBnd:
                    if (isPole == Location::Null)
                        area -= calcCorrectArea(x(2), x(1), normVectors(2), 1);
                    area += calcCorrectArea(x1, x(0), normVectors(0), -1);
                    break;
                case NorthBnd:
                    if (isPole == Location::Null) {
                        area -= calcCorrectArea(x0, x(2), normVectors(3), 1);
                        area += calcCorrectArea(x(0), x(1), normVectors(1), -1);
                    } else if (isPole == Location::SouthPole)
                        area -= calcCorrectArea(x0, x(1), normVectors(2), 1);
                    break;
                case SouthBnd:
                    if (isPole == Location::Null) {
                        area -= calcCorrectArea(x(1), x(0), normVectors(1), 1);
                        area += calcCorrectArea(x(2), x0, normVectors(3), -1);
                    } else if (isPole == Location::NorthPole)
                        area += calcCorrectArea(x(1), x0, normVectors(2), -1);
                    break;
                default:
                    REPORT_ERROR("Unknown boundary!");
            }
            break;
        case 4:
            switch (from) {
                case EastBnd:
                    if (isPole == Location::Null) {
                        area -= calcCorrectArea(x(1), x(0), normVectors(1), 1);
                        area += calcCorrectArea(x(2), x(3), normVectors(3), -1);
                    } else if (isPole == Location::NorthPole) {
                        area += calcCorrectArea(x(1), x(2), normVectors(2), -1);
                    } else if (isPole == Location::SouthPole)
                        area -= calcCorrectArea(x(1), x(0), normVectors(1), 1);
                    break;
                case WestBnd:
                    if (isPole == Location::Null) {
                        area -= calcCorrectArea(x(3), x(2), normVectors(3), 1);
                        area += calcCorrectArea(x(0), x(1), normVectors(1), -1);
                    } else if (isPole == Location::NorthPole) {
                        area += calcCorrectArea(x(0), x(1), normVectors(1), -1);
                    } else if (isPole == Location::SouthPole)
                        area -= calcCorrectArea(x(2), x(1), normVectors(2), 1);
                    break;
                case NorthBnd:
                    if (isPole == Location::Null) {
                        area -= calcCorrectArea(x(0), x1, normVectors(0), 1);
                        area += calcCorrectArea(x(1), x(2), normVectors(2), -1);
                        area -= calcCorrectArea(x0, x(3), normVectors(4), 1);
                    } else if (isPole == Location::SouthPole) {
                        area -= calcCorrectArea(x(0), x1, normVectors(0), 1);
                        area -= calcCorrectArea(x0, x(2), normVectors(3), 1);
                    }
                    break;
                case SouthBnd:
                    if (isPole == Location::Null) {
                        area += calcCorrectArea(x1, x(0), normVectors(0), -1);
                        area -= calcCorrectArea(x(2), x(1), normVectors(2), 1);
                        area += calcCorrectArea(x(3), x0, normVectors(4), -1);
                    } else if (isPole == Location::NorthPole) {
                        area += calcCorrectArea(x1, x(0), normVectors(0), -1);
                        area += calcCorrectArea(x(2), x0, normVectors(3), -1);
                    }
                    break;
                default:
                    REPORT_ERROR("Unknown boundary!");
            }
    }
    // -------------------------------------------------------------------------
    //if (area/Sphere::radius2 < 1.0e-6 || (area < 0.0 && fabs(area) < 1.0e-6))
    if (area < 0.0)
        area = 0.0;
    return area;
}

void MeshAdaptor::recordOverlapArea(double cellArea, int I, int J,
                                    Bnd from, Bnd to, int bndDiff,
                                    Polygon *polygon,
                                    double area, double &totalArea,
                                    list<OverlapArea *> &overlapAreas)
{
    // check multiply entried cell
    list<OverlapArea>::iterator it = overlapAreaList(I, J, 0).begin();
    for (; it != overlapAreaList(I, J, 0).end(); ++it) {
        if ((*it).polygon == polygon) {
            totalArea -= (*it).area;
            double testArea = (*it).area+area-cellArea;
            if (testArea > 0.0)
                (*it).area = testArea;
            else
                (*it).area += area;
            totalArea += (*it).area;
            return;
        }
    }
    OverlapArea overlapArea;
    overlapArea.polygon = polygon;
    overlapArea.area = area;
    overlapAreaList(I, J, 0).push_back(overlapArea);
    totalArea += area;
    overlapAreas.push_back(&overlapAreaList(I, J, 0).back());
}

void MeshAdaptor::recordOverlapArea(double cellArea, int I, int J,
                                    Polygon *polygon,
                                    double area, double &totalArea,
                                    list<OverlapArea *> &overlapAreas)
{
    // check multiply entried cell
    list<OverlapArea>::iterator it = overlapAreaList(I, J, 0).begin();
    for (; it != overlapAreaList(I, J, 0).end(); ++it) {
        if ((*it).polygon == polygon) {
            totalArea -= (*it).area;
            double testArea = (*it).area+area-cellArea;
            if (testArea > 0.0)
                (*it).area = testArea;
            else
                (*it).area += area;
            totalArea += (*it).area;
            return;
        }
    }
    OverlapArea overlapArea;
    overlapArea.polygon = polygon;
    overlapArea.area = area;
    overlapAreaList(I, J, 0).push_back(overlapArea);
    totalArea += area;
    overlapAreas.push_back(&overlapAreaList(I, J, 0).back());
}

void MeshAdaptor::adapt(const TracerManager &tracerManager,
                        const MeshManager &meshManager)
{
    NOTICE("MeshAdaptor::adapt", "running ...");
    const RLLMesh &mesh = meshManager.getMesh(PointCounter::Bound);
    int numLon = mesh.getNumLon()-2;
    int numLat = mesh.getNumLat()-1;
    const double areaDiffThreshold = 1.0e-2;
    double totalArea, realArea, diffArea, maxDiffArea = 0.0;
    map<int, list<int> > bndCellIdx;
    list<OverlapArea *> overlapAreas;
    static CoverMask coverMask(meshManager.getMesh(PointCounter::Center));
    // -------------------------------------------------------------------------
    // reset
    for (int i = 0; i < overlapAreaList.extent(0); ++i)
        for (int j = 0; j < overlapAreaList.extent(1); ++j)
            overlapAreaList(i, j, 0).clear();
    // -------------------------------------------------------------------------
    // calculate the overlap area between polygon and mesh
    Polygon *polygon = tracerManager.polygonManager.polygons.front();
    for (int m = 0; m < tracerManager.polygonManager.polygons.size(); ++m) {
#ifdef DEBUG
        bool debug = false;
        int counter = 0;
//        if (TimeManager::getSteps() == 2 && polygon->getID() == 17285) {
//            polygon->dump("polygon");
//            REPORT_DEBUG;
//            debug = true;
//        }
#endif
        // record the previous edge and intersection
        EdgePointer *edgePointer0 = NULL; Coordinate x0;
        // record the starting edge and intersection
        EdgePointer *edgePointer00 = NULL; Coordinate x00;
        // record the coming and going boundary
        Bnd from0, to0, to00, from = NullBnd;
        // record the previous cell index
        int I0, J0;
        // reset
        totalArea = 0.0; realArea = polygon->getArea(NewTimeLevel);
        bndCellIdx.clear();
        overlapAreas.clear();
        // internal variables
        int I, J, I1, I2, J1, J2, bndDiff;
        double lonBnd1, lonBnd2, latBnd1, latBnd2;
        Coordinate x;
        // Note: The calculation of intersection between great-circle arc and
        //       latitudinal line may fail under normal double precision
        //       floating-point calculation, so swith to MPFR if necessary.
        bool useMPFR;
        // ---------------------------------------------------------------------
        // search overlapped mesh cell along polygon edges
        EdgePointer *edgePointer = polygon->edgePointers.front();
        for (int n = 0; n < polygon->edgePointers.size(); ++n) {
            Vertex *vertex1 = edgePointer->getEndPoint(FirstPoint);
            Vertex *vertex2 = edgePointer->getEndPoint(SecondPoint);
            const Coordinate &x1 = vertex1->getCoordinate();
            const Coordinate &x2 = vertex2->getCoordinate();
            I1 = vertex1->getLocation().i[4];
            J1 = vertex1->getLocation().j[4];
            I2 = vertex2->getLocation().i[4];
            J2 = vertex2->getLocation().j[4];
            // start from the cell where the first point is at
            I = I1, J = J1, I0 = I1, J0 = J1;
            while (true) {
                useMPFR = false;
                lonBnd1 = mesh.lon(I);
                lonBnd2 = mesh.lon(I+1);
                latBnd1 = mesh.lat(J);
                latBnd2 = mesh.lat(J+1);
                // check if get into the cell where the second point is
                if (I == I2 && J == J2) break;
                // record boundary cell indices
                bndCellIdx[I].push_back(J);
                // Note: There are four directions to search.
                // western boundary
                if (from != WestBnd || edgePointer != edgePointer0) {
                    if (Sphere::calcIntersectLat(x1, x2, lonBnd1,
                                                 latBnd1, latBnd2, x)) {
                        I0 = I; J0 = J;
                        from0 = from; to0 = WestBnd;;
                        I = I-1; if (I == -1) I = numLon-1;
                        from = EastBnd;
                        goto calc_overlap_area;
                    }
                }
                // eastern boundary
                if (from != EastBnd || edgePointer != edgePointer0) {
                    if (Sphere::calcIntersectLat(x1, x2, lonBnd2,
                                                 latBnd1, latBnd2, x)) {
                        I0 = I; J0 = J;
                        from0 = from; to0 = EastBnd;
                        I = I+1; if (I == numLon) I = 0;
                        from = WestBnd;
                        goto calc_overlap_area;
                    }
                }
            calculate_use_mpfr:
                // northern boundary
                if ((from != NorthBnd && J > 0) ||
                    edgePointer != edgePointer0) {
                    if (Sphere::calcIntersectLon(x1, x2, lonBnd1, lonBnd2,
                                                 latBnd1, x, useMPFR)) {
                        I0 = I; J0 = J;
                        from0 = from; to0 = NorthBnd;
                        J = J-1;
                        from = SouthBnd;
                        goto calc_overlap_area;
                    }
                }
                // southern boundary
                if ((from != SouthBnd && J < numLat) ||
                    edgePointer != edgePointer0) {
                    if (Sphere::calcIntersectLon(x1, x2, lonBnd1, lonBnd2,
                                                 latBnd2, x, useMPFR)) {
                        I0 = I; J0 = J;
                        from0 = from; to0 = SouthBnd;
                        J = J+1;
                        from = NorthBnd;
                        goto calc_overlap_area;
                    }
                }
                // -------------------------------------------------------------
                // Note: Here no intersection has been found under the normal
                //       double precision floating point calculation, so switch
                //       to high precision MPFR version.
                if (useMPFR) {
                    Message message;
                    message << "Intersection can not be found for polygon ";
                    message << polygon->getID() << "!";
                    REPORT_ERROR(message.str());
                }
                useMPFR = true;
                goto calculate_use_mpfr;
            calc_overlap_area:
                if (edgePointer0 != NULL) {
                    double area = calcOverlapArea(I0, J0, from0, to0, bndDiff,
                                                  lonBnd1, lonBnd2,
                                                  latBnd1, latBnd2,
                                                  x0, edgePointer0,
                                                  x, edgePointer);
                    recordOverlapArea(mesh.area(I0, J0), I0, J0,
                                      from0, to0, bndDiff,
                                      polygon, area, totalArea, overlapAreas);
                }
                // record the starting edge and intersection
                if (edgePointer00 == NULL) {
                    x00 = x; edgePointer00 = edgePointer; to00 = to0;
                }
                // record the previous edge and intersection
                x0 = x; edgePointer0 = edgePointer;
#ifdef DEBUG
                counter++;
#endif
            }
            edgePointer = edgePointer->next;
        }
        if (edgePointer00 != NULL) {
            double area = calcOverlapArea(I, J, from, to00, bndDiff,
                                          lonBnd1, lonBnd2, latBnd1, latBnd2,
                                          x0, edgePointer0, x00, edgePointer00);
            recordOverlapArea(mesh.area(I, J), I, J, from, to00, bndDiff,
                              polygon, area, totalArea, overlapAreas);
        } else
            recordOverlapArea(mesh.area(I, J), I, J, polygon,
                              polygon->getArea(NewTimeLevel),
                              totalArea, overlapAreas);
        // ---------------------------------------------------------------------
        // check if pole has been included
        diffArea = fabs(totalArea-realArea)/realArea;
        if (diffArea > areaDiffThreshold) {
            Location::Pole checkPole = Location::Null;
            // Note: Here we assume that if the boundary cells cover the whole
            //       zonal range, then the pole has been included
            if (bndCellIdx.size() == numLon) {
                checkPole = 
                polygon->edgePointers.front()->getEndPoint(FirstPoint)->
                getCoordinate().getLat() > 0.0 ?
                Location::NorthPole : Location::SouthPole;
            }
#ifdef DEBUG
            coverMask.init(polygon, bndCellIdx, checkPole, mesh, debug);
#else
            coverMask.init(polygon, bndCellIdx, checkPole, mesh);
#endif
            // record the fully covered cells from pole to the first or last
            // crossed cell along longitude
            if (checkPole != Location::Null)
                for (int i = 0; i < coverMask.mask.extent(0); ++i)
                    for (int j = 0; j < coverMask.mask.extent(1); ++j)
                        if (coverMask.mask(i, j) ==
                            CoverMask::FullyCoveredNearPole)
                            recordOverlapArea(mesh.area(coverMask.idxI(i),
                                                        coverMask.idxJ(j)),
                                              coverMask.idxI(i),
                                              coverMask.idxJ(j), polygon,
                                              mesh.area(coverMask.idxI(i),
                                                        coverMask.idxJ(j)),
                                              totalArea, overlapAreas);
            diffArea = fabs(totalArea-realArea)/realArea;
        }
        // ---------------------------------------------------------------------
        // handle the cells that are fully covered by the polygon
        if (coverMask.mask.size() != 0 && any(coverMask.mask == -1)) {
#ifdef DEBUG
            coverMask.searchCover(polygon, debug);
#else
            coverMask.searchCover(polygon);
#endif
            // add the fully covered cells
            for (int i = 0; i < coverMask.mask.extent(0); ++i)
                for (int j = 0; j < coverMask.mask.extent(1); ++j)
                    if (coverMask.mask(i, j) == CoverMask::FullyCovered)
                        recordOverlapArea(mesh.area(coverMask.idxI(i),
                                                    coverMask.idxJ(j)),
                                          coverMask.idxI(i),
                                          coverMask.idxJ(j), polygon,
                                          mesh.area(coverMask.idxI(i),
                                                    coverMask.idxJ(j)),
                                          totalArea, overlapAreas);
            diffArea = fabs(totalArea-realArea)/realArea;
        }
        // ---------------------------------------------------------------------
        // record totalArea in each overlapArea to overcome numerical inaccuracy
        list<OverlapArea *>::const_iterator itOaPtr = overlapAreas.begin();
        for (; itOaPtr != overlapAreas.end(); ++itOaPtr)
            (*itOaPtr)->totalArea = totalArea;
        // ---------------------------------------------------------------------
        if (diffArea > areaDiffThreshold) {
            ostringstream message;
            message << "Failed to calculate overlap area for polygon ";
            message << polygon->getID() << "!" << endl;
            polygon->dump("polygon");
            REPORT_ERROR(message.str());
        }
        maxDiffArea = fmax(maxDiffArea, diffArea);
        polygon = polygon->next;
    }
#ifdef DEBUG
    cout << "Maximum area difference: " << maxDiffArea*100 << "%" << endl;
#endif
}

void MeshAdaptor::remap(const string &tracerName, const Field &q,
                        TracerManager &tracerManager)
{
    NOTICE("MeshAdaptor::remap", "Remapping "+tracerName+" onto polygons ...");
    int tracerId = tracerManager.getTracerId(tracerName);
    const RLLMesh &mesh = q.getMesh(Field::Bound);
    // -------------------------------------------------------------------------
    // reset tracer mass
    Polygon *polygon = tracerManager.polygonManager.polygons.front();
    for (int i = 0; i < tracerManager.polygonManager.polygons.size(); ++i) {
        if (polygon->tracers.size() == 0)
            // new polygon
            polygon->tracers.resize(tracerManager.getTracerNum());
        polygon->tracers[tracerId].setMass(0.0);
        polygon = polygon->next;
    }
    // -------------------------------------------------------------------------
    // accumulate tracer mass
    double totalCellMass = 0.0, totalPolygonMass = 0.0;
    for (int i = 0; i < overlapAreaList.extent(0); ++i)
        for (int j = 0; j < overlapAreaList.extent(1); ++j) {
            double totalArea = 0.0;
            double cellMass = q(i, j).getNew()*mesh.area(i, j);
            totalCellMass += cellMass;
            list<OverlapArea>::const_iterator itOa;
            // accumulate overlap area
            for (itOa = overlapAreaList(i, j, 0).begin();
                 itOa != overlapAreaList(i, j, 0).end(); ++itOa)
                totalArea += (*itOa).area;
            // use partition of unity to ensure exact mass conservation since
            // there are numerical errors when calculate overlap area
            for (itOa = overlapAreaList(i, j, 0).begin();
                 itOa != overlapAreaList(i, j, 0).end(); ++itOa) {
                double weight = (*itOa).area/totalArea;
                (*itOa).polygon->tracers[tracerId].addMass(cellMass*weight);
            }
        }
    // -------------------------------------------------------------------------
    // calculate tracer density
    polygon = tracerManager.polygonManager.polygons.front();
    for (int i = 0; i < tracerManager.polygonManager.polygons.size(); ++i) {
        polygon->updateTracer(tracerId);
        totalPolygonMass += polygon->tracers[tracerId].getMass();
        polygon = polygon->next;
    }
    // -------------------------------------------------------------------------
    cout << "Total cell mass is    " << setprecision(20) << totalCellMass << endl;
    cout << "Total polygon mass is " << setprecision(20) << totalPolygonMass << endl;
    double massError = (totalCellMass-totalPolygonMass)/totalPolygonMass;
    cout << "Mass error is " << massError << "%" << endl;
    if (fabs(massError) > 1.0e-10)
        REPORT_ERROR("Mass error is too large!");
}

void MeshAdaptor::remap(const string &tracerName, TracerManager &tracerManager)
{
    NOTICE("MeshAdaptor::remap", "Remapping "+tracerName+" onto mesh ...");
    int tracerId = tracerManager.getTracerId(tracerName);
    Field &q = tracerManager.getTracerDensityField(tracerId);
    const RLLMesh &mesh = q.getMesh(Field::Bound);
    // -------------------------------------------------------------------------
    double totalCellMass = 0.0, totalPolygonMass = 0.0;
    for (int i = 0; i < overlapAreaList.extent(0); ++i)
        for (int j = 0; j < overlapAreaList.extent(1); ++j) {
            q(i, j) = 0.0;
            list<OverlapArea>::const_iterator itOa;
            for (itOa = overlapAreaList(i, j, 0).begin();
                 itOa != overlapAreaList(i, j, 0).end(); ++itOa) {
                double weight = (*itOa).area/(*itOa).totalArea;
                q(i, j) += (*itOa).polygon->tracers[tracerId].getMass()*weight;
            }
            totalCellMass += q(i, j).getNew();
            q(i, j) /= mesh.area(i, j);
        }
    // -------------------------------------------------------------------------
    Polygon *polygon = tracerManager.polygonManager.polygons.front();
    for (int i = 0; i < tracerManager.polygonManager.polygons.size(); ++i) {
        totalPolygonMass += polygon->tracers[tracerId].getMass();
        polygon = polygon->next;
    }
    // -------------------------------------------------------------------------
    cout << "Total cell mass is    " << setprecision(20) << totalCellMass << endl;
    cout << "Total polygon mass is " << setprecision(20) << totalPolygonMass << endl;
    double massError = (totalPolygonMass-totalCellMass)/totalCellMass;
    cout << "Mass error is " << massError << "%" << endl;
    if (fabs(massError) > 1.0e-10)
        REPORT_ERROR("Mass error is too large!");
}
