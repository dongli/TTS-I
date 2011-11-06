#include "MeshAdaptor.h"
#include "ReportMacros.h"
#include "Sphere.h"

#include <map>
#include <list>
#ifdef DEBUG
#include <fstream>
using std::ofstream;
#endif

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
    x3.set(x1.getLon(), 0.0);
    x4.set(x2.getLon(), 0.0);

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
                angle1 = EdgePointer::calcAngle(normVector, normVector1, x1);
                angle2 = EdgePointer::calcAngle(normVector2, normVector, x2);
                break;
            case -1:
                angle1 = EdgePointer::calcAngle(-normVector, normVector1, x1);
                angle2 = EdgePointer::calcAngle(normVector2, -normVector, x2);
                break;
            default:
                REPORT_ERROR("Unknown sign flag!");
        }
    else
        switch (signFlag) {
            case 1:
                angle1 = EdgePointer::calcAngle(normVector1, normVector, x1);
                angle2 = EdgePointer::calcAngle(normVector, normVector2, x2);
                break;
            case -1:
                angle1 = EdgePointer::calcAngle(normVector1, -normVector, x1);
                angle2 = EdgePointer::calcAngle(-normVector, normVector2, x2);
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
                                    const Coordinate &x0,
                                    EdgePointer *edgePointer0,
                                    const Coordinate &x1,
                                    EdgePointer *edgePointer1)
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
    Vector *normVectors;
    Coordinate *x;
    double *angles, excess, area;

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

    // -------------------------------------------------------------------------
    // get the normal vector of the cell edges
    normVectors = new Vector[numCellEdge];
    x = new Coordinate[numCellEdge-1];
    switch (bndDiff) {
        case 0:
            break;
        case 1:
            switch (from) {
                case EastBnd:
                    if (isPole != Location::SouthPole)
                        x[0].set(lonBnd2, latBnd2);
                    break;
                case WestBnd:
                    if (isPole != Location::NorthPole)
                        x[0].set(lonBnd1, latBnd1);
                    break;
                case NorthBnd:
                    if (isPole != Location::NorthPole)
                        x[0].set(lonBnd2, latBnd1);
                    break;
                case SouthBnd:
                    if (isPole != Location::SouthPole)
                        x[0].set(lonBnd1, latBnd2);
                    break;
                default:
                    REPORT_ERROR("Unknown boundary!");
            }
            break;
        case 2:
            switch (from) {
                case EastBnd:
                    if (isPole != Location::SouthPole) {
                        x[0].set(lonBnd1, latBnd2);
                        x[1].set(lonBnd2, latBnd2);
                    } else
                        x[0].set(0.0, -PI05);
                    break;
                case WestBnd:
                    if (isPole != Location::NorthPole) {
                        x[0].set(lonBnd2, latBnd1);
                        x[1].set(lonBnd1, latBnd1);
                    } else
                        x[0].set(0.0, PI05);
                    break;
                case NorthBnd:
                    x[0].set(lonBnd2, latBnd2);
                    x[1].set(lonBnd2, latBnd1);
                    break;
                case SouthBnd:
                    x[0].set(lonBnd1, latBnd1);
                    x[1].set(lonBnd1, latBnd2);
                    break;
                default:
                    REPORT_ERROR("Unknown boundary!");
            }
            break;
        case -1:
            switch (from) {
                case EastBnd:
                    if (isPole == Location::Null) {
                        x[0].set(lonBnd1, latBnd1);
                        x[1].set(lonBnd1, latBnd2);
                        x[2].set(lonBnd2, latBnd2);
                    } else if (isPole == Location::SouthPole) {
                        x[0].set(lonBnd1, latBnd1);
                        x[1].set(0.0, -PI05);
                    }
                    break;
                case WestBnd:
                    if (isPole == Location::Null) {
                        x[0].set(lonBnd2, latBnd2);
                        x[1].set(lonBnd2, latBnd1);
                        x[2].set(lonBnd1, latBnd1);
                    } else if (isPole == Location::NorthPole) {
                        x[0].set(lonBnd2, latBnd2);
                        x[1].set(0.0, PI05);
                    }
                    break;
                case NorthBnd:
                    if (isPole == Location::Null) {
                        x[0].set(lonBnd1, latBnd2);
                        x[1].set(lonBnd2, latBnd2);
                        x[2].set(lonBnd2, latBnd1);
                    } else if (isPole == Location::SouthPole) {
                        x[0].set(0.0, -PI05);
                        x[1].set(lonBnd2, latBnd1);
                    }
                    break;
                case SouthBnd:
                    if (isPole == Location::Null) {
                        x[0].set(lonBnd2, latBnd1);
                        x[1].set(lonBnd1, latBnd1);
                        x[2].set(lonBnd1, latBnd2);
                    } else if (isPole == Location::NorthPole) {
                        x[0].set(0.0, PI05);
                        x[1].set(lonBnd1, latBnd2);
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
                        x[0].set(lonBnd2, latBnd1);
                        x[1].set(lonBnd1, latBnd1);
                        x[2].set(lonBnd1, latBnd2);
                        x[3].set(lonBnd2, latBnd2);
                    } else if (isPole == Location::NorthPole) {
                        x[0].set(0.0, PI05);
                        x[1].set(lonBnd1, latBnd2);
                        x[2].set(lonBnd2, latBnd2);
                    } else if (isPole == Location::SouthPole) {
                        x[0].set(lonBnd2, latBnd1);
                        x[1].set(lonBnd1, latBnd1);
                        x[2].set(0.0, -PI05);
                    }
                    break;
                case WestBnd:
                    if (isPole == Location::Null) {
                        x[0].set(lonBnd1, latBnd2);
                        x[1].set(lonBnd2, latBnd2);
                        x[2].set(lonBnd2, latBnd1);
                        x[3].set(lonBnd1, latBnd1);
                    } else if (isPole == Location::NorthPole) {
                        x[0].set(lonBnd1, latBnd2);
                        x[1].set(lonBnd2, latBnd2);
                        x[2].set(0.0, PI05);
                    } else if (isPole == Location::SouthPole) {
                        x[0].set(0.0, -PI05);
                        x[1].set(lonBnd2, latBnd1);
                        x[2].set(lonBnd1, latBnd1);
                    }
                    break;
                case NorthBnd:
                    if (isPole == Location::Null) {
                        x[0].set(lonBnd1, latBnd1);
                        x[1].set(lonBnd1, latBnd2);
                        x[2].set(lonBnd2, latBnd2);
                        x[3].set(lonBnd2, latBnd1);
                    } else if (isPole == Location::SouthPole) {
                        x[0].set(lonBnd1, latBnd1);
                        x[1].set(0.0, -PI05);
                        x[2].set(lonBnd2, latBnd1);
                    }
                    break;
                case SouthBnd:
                    if (isPole == Location::Null) {
                        x[0].set(lonBnd2, latBnd2);
                        x[1].set(lonBnd2, latBnd1);
                        x[2].set(lonBnd1, latBnd1);
                        x[3].set(lonBnd1, latBnd2);
                    } else if (isPole == Location::NorthPole) {
                        x[0].set(lonBnd2, latBnd2);
                        x[1].set(0.0, PI05);
                        x[2].set(lonBnd1, latBnd2);
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
        if (x1.getLon() != x[0].getLon() || x1.getLat() != x[0].getLat()) {
            normVectors[0] = norm_cross(x[0].getCAR(), x1.getCAR());
            for (i = 1; i < numCellEdge-1; ++i)
                normVectors[i] = norm_cross(x[i].getCAR(), x[i-1].getCAR());
        } else {
            numCellEdge--;
            for (i = 0; i < numCellEdge-1; ++i)
                normVectors[i] = norm_cross(x[i+1].getCAR(), x[i].getCAR());
        }
        if (x0.getLon() != x[i-1].getLon() || x0.getLat() != x[i-1].getLat())
            normVectors[i] = norm_cross(x0.getCAR(), x[i-1].getCAR());
        else
            numCellEdge--;
    } else
        normVectors[0] = norm_cross(x0.getCAR(), x1.getCAR());

    // -------------------------------------------------------------------------
    // get or calculate the internal angles
    numEdge = numCellEdge+numPolygonEdge;
    angles = new double[numEdge];

    edgePointer = edgePointer0->next;
    for (i = 0; i < numPolygonEdge-1; ++i) {
        angles[i] = edgePointer->getAngle(NewTimeLevel);
        edgePointer = edgePointer->next;
    }

    angles[i++] = EdgePointer::calcAngle(normVector1, normVectors[0], x1);
    j = 0;
    for (; i < numEdge-1; ++i) {
        angles[i] = EdgePointer::calcAngle(normVectors[j],
                                           normVectors[j+1], x[j]);
        j++;
    }
    angles[i] = EdgePointer::calcAngle(normVectors[j], normVector0, x0);

    // -------------------------------------------------------------------------
    // numerical tolerance
    const double smallAngle = 1.0/Rad2Deg;
    if (PI2-angles[numPolygonEdge-1] < smallAngle ||
        PI2-angles[numEdge-1] < smallAngle) {
        return 0.0;
    }

    // -------------------------------------------------------------------------
    // calculate the area of overlapping polygon (assuming all great circle
    // arc edges)
    excess = 0.0;
    for (i = 0; i < numEdge; ++i)
        excess += angles[i];
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
                    area -= calcCorrectArea(x0, x1, normVectors[0], 1);
                    break;
                case SouthBnd:
                    area += calcCorrectArea(x1, x0, normVectors[0], -1);
                    break;
                default:
                    REPORT_ERROR("Unknown boundary!");
            }
            break;
        case 1:
            switch (from) {
                case EastBnd:
                    if (isPole != Location::SouthPole)
                        area += calcCorrectArea(x1, x[0], normVectors[0], -1);
                    break;
                case WestBnd:
                    if (isPole != Location::NorthPole)
                        area -= calcCorrectArea(x[0], x1, normVectors[0], 1);
                    break;
                case NorthBnd:
                    if (isPole != Location::NorthPole)
                        area -= calcCorrectArea(x0, x[0], normVectors[1], 1);
                    break;
                case SouthBnd:
                    if (isPole != Location::SouthPole)
                        area += calcCorrectArea(x[0], x0, normVectors[1], -1);
                    break;
                default:
                    REPORT_ERROR("Unknown boundary!");
            }
            break;
        case 2:
            switch (from) {
                case EastBnd:
                    if (isPole != Location::SouthPole)
                        area += calcCorrectArea(x[0], x[1], normVectors[1], -1);
                    break;
                case WestBnd:
                    if (isPole != Location::NorthPole)
                        area -= calcCorrectArea(x[1], x[0], normVectors[1], 1);
                    break;
                case NorthBnd:
                    area -= calcCorrectArea(x0, x[1], normVectors[2], 1);
                    area += calcCorrectArea(x1, x[0], normVectors[0], -1);
                    break;
                case SouthBnd:
                    area -= calcCorrectArea(x[0], x1, normVectors[0], 1);
                    area += calcCorrectArea(x[1], x0, normVectors[2], -1);
                    break;
                default:
                    REPORT_ERROR("Unknown boundary!");
            }
            break;
        case -1:
            switch (from) {
                case EastBnd:
                    area -= calcCorrectArea(x[0], x1, normVectors[0], 1);
                    if (isPole == Location::Null)
                        area += calcCorrectArea(x[1], x[2], normVectors[2], -1);
                    break;
                case WestBnd:
                    if (isPole == Location::Null)
                        area -= calcCorrectArea(x[2], x[1], normVectors[2], 1);
                    area += calcCorrectArea(x1, x[0], normVectors[0], -1);
                    break;
                case NorthBnd:
                    if (isPole == Location::Null) {
                        area -= calcCorrectArea(x0, x[2], normVectors[3], 1);
                        area += calcCorrectArea(x[0], x[1], normVectors[1], -1);
                    } else if (isPole == Location::SouthPole)
                        area -= calcCorrectArea(x0, x[1], normVectors[2], 1);
                    break;
                case SouthBnd:
                    if (isPole == Location::Null) {
                        area -= calcCorrectArea(x[1], x[0], normVectors[1], 1);
                        area += calcCorrectArea(x[2], x0, normVectors[3], -1);
                    } else if (isPole == Location::NorthPole)
                        area += calcCorrectArea(x[1], x0, normVectors[2], -1);
                    break;
                default:
                    REPORT_ERROR("Unknown boundary!");
            }
            break;
        case 4:
            switch (from) {
                case EastBnd:
                    if (isPole == Location::Null) {
                        area -= calcCorrectArea(x[1], x[0], normVectors[1], 1);
                        area += calcCorrectArea(x[2], x[3], normVectors[3], -1);
                    } else if (isPole == Location::NorthPole) {
                        area += calcCorrectArea(x[1], x[2], normVectors[2], -1);
                    } else if (isPole == Location::SouthPole)
                        area -= calcCorrectArea(x[1], x[0], normVectors[1], 1);
                    break;
                case WestBnd:
                    if (isPole == Location::Null) {
                        area -= calcCorrectArea(x[3], x[2], normVectors[3], 1);
                        area += calcCorrectArea(x[0], x[1], normVectors[1], -1);
                    } else if (isPole == Location::NorthPole) {
                        area += calcCorrectArea(x[0], x[1], normVectors[1], -1);
                    } else if (isPole == Location::SouthPole)
                        area -= calcCorrectArea(x[2], x[1], normVectors[2], 1);
                    break;
                case NorthBnd:
                    if (isPole == Location::Null) {
                        area -= calcCorrectArea(x[0], x1, normVectors[0], 1);
                        area += calcCorrectArea(x[1], x[2], normVectors[2], -1);
                        area -= calcCorrectArea(x0, x[3], normVectors[4], 1);
                    } else if (isPole == Location::SouthPole) {
                        area -= calcCorrectArea(x[0], x1, normVectors[0], 1);
                        area -= calcCorrectArea(x0, x[2], normVectors[3], 1);
                    }
                    break;
                case SouthBnd:
                    if (isPole == Location::Null) {
                        area += calcCorrectArea(x1, x[0], normVectors[0], -1);
                        area -= calcCorrectArea(x[2], x[1], normVectors[2], 1);
                        area += calcCorrectArea(x[3], x0, normVectors[4], -1);
                    } else if (isPole == Location::NorthPole) {
                        area += calcCorrectArea(x1, x[0], normVectors[0], -1);
                        area += calcCorrectArea(x[2], x0, normVectors[3], -1);
                    }
                    break;
                default:
                    REPORT_ERROR("Unknown boundary!");
            }
    }

    // -------------------------------------------------------------------------
    delete [] normVectors;
    delete [] x;
    delete [] angles;

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

inline bool isCellCovered(int numCellLon, int numCellLat,
                          Array<int, 2> &coverMask,
                          int i0, int j0, int i, int j)
{
    // mask meanings:
    //  0 - cell is not covered by polygon
    //  1 - cell is partially covered by polygon
    //  2 - cell is fully covered by polygon
    // -1 - cell is potentially fully covered by polygon
    // -------------------------------------------------------------------------
    if (coverMask(i, j) > 0)
        return true;
    else if (coverMask(i, j)== 0)
        return false;
    // -------------------------------------------------------------------------
    int ii, jj;
    // check left cells
    for (ii = i-1; ii >= 0; --ii)
        if (coverMask(ii, j) > 0)
            break;
        else if (coverMask(ii, j) == 0)
            goto return_not_covered;
        else if (coverMask(ii, j) == -1 && ii == i0 && j == j0)
            break;
    if (ii == -1)
        goto return_not_covered;
    // check right cells
    for (ii = i+1; ii < numCellLon; ++ii)
        if (coverMask(ii, j) > 0)
            break;
        else if (coverMask(ii, j) == 0)
            goto return_not_covered;
        else if (coverMask(ii, j) == -1)
            if (!isCellCovered(numCellLon, numCellLat, coverMask, i, j, ii, j))
                goto return_not_covered;
            else
                break;
    if (ii == numCellLon)
        goto return_not_covered;
    // check left-top cells
    for (ii = i-1, jj = j-1; ii >= 0 && jj >= 0; --ii, --jj)
        if (coverMask(ii, jj) > 0)
            break;
        else if (coverMask(ii, jj) == 0)
            goto return_not_covered;
        else if (coverMask(ii, jj) == -1 && ii == i0 && jj == j0)
            break;
    if (ii == -1 || jj == -1)
        goto return_not_covered;
    // check right-bottom cells
    for (ii = i+1, jj = j+1; ii < numCellLon && jj < numCellLat; ++ii, ++jj)
        if (coverMask(ii, jj) > 0)
            break;
        else if (coverMask(ii, jj) == 0)
            goto return_not_covered;
        else if (coverMask(ii, jj) == -1)
            if (!isCellCovered(numCellLon, numCellLat, coverMask, i, j, ii, jj))
                goto return_not_covered;
            else
                break;
    if (ii == numCellLon || jj == numCellLat)
        goto return_not_covered;
    // check left-bottom cells
    for (ii = i-1, jj = j+1; ii >= 0 && jj < numCellLat; --ii, ++jj)
        if (coverMask(ii, jj) > 0)
            break;
        else if (coverMask(ii, jj) == 0)
            goto return_not_covered;
        else if (coverMask(ii, jj) == -1 && ii == i0 && jj == j0)
            break;
    if (ii == -1 || jj == numCellLat)
        goto return_not_covered;
    // check right-top cells
    for (ii = i+1, jj = j-1; ii < numCellLon && j >= 0; ++ii, --jj)
        if (coverMask(ii, jj) > 0)
            break;
        else if (coverMask(ii, jj) == 0)
            goto return_not_covered;
        else if (coverMask(ii, jj) == -1)
            if (!isCellCovered(numCellLon, numCellLat, coverMask, i, j, ii, jj))
                goto return_not_covered;
            else
                break;
    if (ii == numCellLon || jj == -1)
        goto return_not_covered;
    // -------------------------------------------------------------------------
    coverMask(i, j) = 2;
    return true;
return_not_covered:
    coverMask(i, j) = 0;
    return false;
}

//#define DEBUG_INTERSECTION

void MeshAdaptor::adapt(const TracerManager &tracerManager,
                        const MeshManager &meshManager)
{
    NOTICE("MeshAdaptor::adapt", "running ...");
    const RLLMesh &mesh = meshManager.getMesh(PointCounter::Bound);

    int numLon = mesh.getNumLon()-1;
    int numLat = mesh.getNumLat()-1;

    const double areaDiffThreshold = 5.0e-3;
    double totalArea, realArea, diffArea, maxDiffArea = 0.0;
    map<int, list<int> > bndCellIdx;
    list<OverlapArea *> overlapAreas;

    // reset
    for (int i = 0; i < overlapAreaList.extent(0); ++i)
        for (int j = 0; j < overlapAreaList.extent(1); ++j)
            overlapAreaList(i, j, 0).clear();

    // calculate the overlap area between polygon and mesh
    Polygon *polygon = tracerManager.polygonManager.polygons.front();
    for (int m = 0; m < tracerManager.polygonManager.polygons.size(); ++m) {
#ifdef DEBUG
        bool debug = false;
        int counter = 0;
//        if (TimeManager::getSteps() == 0 && polygon->getID() == 1) {
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
        Coordinate *x;
        bool isForked = false;
        // ---------------------------------------------------------------------
        // search overlapped mesh cell along polygon edges
#ifdef DEBUG_INTERSECTION
        ofstream file("debug_intersection");
#endif
        EdgePointer *edgePointer = polygon->edgePointers.front();
        for (int n = 0; n < polygon->edgePointers.size(); ++n) {
            Vertex *vertex1 = edgePointer->getEndPoint(FirstPoint);
            Vertex *vertex2 = edgePointer->getEndPoint(SecondPoint);
            const Coordinate &x1 = vertex1->getCoordinate();
            const Coordinate &x2 = vertex2->getCoordinate();
#ifdef DEBUG_INTERSECTION
            file << setw(5) << 0;
            file << setw(30) << setprecision(15) << x1.getLon();
            file << setw(30) << setprecision(15) << x1.getLat() << endl;
#endif
            I1 = vertex1->getLocation().i[4];
            J1 = vertex1->getLocation().j[4];
            I2 = vertex2->getLocation().i[4];
            J2 = vertex2->getLocation().j[4];
            // start from the cell where the first point is at
            I = I1, J = J1, I0 = I1, J0 = J1;
            while (true) {
                lonBnd1 = mesh.lon(I);
                lonBnd2 = mesh.lon(I+1);
                latBnd1 = mesh.lat(J);
                latBnd2 = mesh.lat(J+1);
                // check if get into the cell where the second point is
                if (I == I2 && J == J2) break;
                // record boundary cell indices
                bndCellIdx[I].push_back(J);
                //
                Coordinate x3, x4;
                Vector tmp1, tmp2;
                // Note: There are four directions to search.
                // western boundary
                if (from != WestBnd || edgePointer != edgePointer0) {
                    Sphere::calcIntersectLat(x1, x2, lonBnd1, x3, x4);
                    if (Sphere::is_lon_eq(x3.getLon(), lonBnd1) &&
                        (x3.getLat() >= latBnd2 && x3.getLat() < latBnd1)) {
                        if (dot(x1.getCAR(), x3.getCAR()) > 0.0) {
                            tmp1 = cross(x1.getCAR(), x3.getCAR());
                            tmp2 = cross(x3.getCAR(), x2.getCAR());
                            if (dot(tmp1, tmp2) > 0.0) {
                                I0 = I; J0 = J;
                                from0 = from; to0 = WestBnd; x = &x3;
                                I = I-1; if (I == -1) I = numLon-1;
                                from = EastBnd;
                                goto calc_overlap_area;
                            }
                        }
                    }
                    if (Sphere::is_lon_eq(x4.getLon(), lonBnd1) &&
                        (x4.getLat() >= latBnd2 && x4.getLat() < latBnd1)) {
                        if (dot(x1.getCAR(), x4.getCAR()) > 0.0) {
                            tmp1 = cross(x1.getCAR(), x4.getCAR());
                            tmp2 = cross(x4.getCAR(), x2.getCAR());
                            if (dot(tmp1, tmp2) > 0.0) {
                                I0 = I; J0 = J;
                                from0 = from; to0 = WestBnd; x = &x4;
                                I = I-1; if (I == -1) I = numLon-1;
                                from = EastBnd;
                                goto calc_overlap_area;
                            }
                        }
                    }
                }
                // eastern boundary
                if (from != EastBnd || edgePointer != edgePointer0) {
                    Sphere::calcIntersectLat(x1, x2, lonBnd2, x3, x4);
                    if (Sphere::is_lon_eq(x3.getLon(), lonBnd2) &&
                        (x3.getLat() >= latBnd2 && x3.getLat() < latBnd1)) {
                        if (dot(x1.getCAR(), x3.getCAR()) > 0.0) {
                            tmp1 = cross(x1.getCAR(), x3.getCAR());
                            tmp2 = cross(x3.getCAR(), x2.getCAR());
                            if (dot(tmp1, tmp2) > 0.0) {
                                I0 = I; J0 = J;
                                from0 = from; to0 = EastBnd; x = &x3;
                                I = I+1; if (I == numLon) I = 0;
                                from = WestBnd;
                                goto calc_overlap_area;
                            }
                        }
                    }
                    if (Sphere::is_lon_eq(x4.getLon(), lonBnd2) &&
                        (x4.getLat() >= latBnd2 && x4.getLat() < latBnd1)) {
                        if (dot(x1.getCAR(), x4.getCAR()) > 0.0) {
                            tmp1 = cross(x1.getCAR(), x4.getCAR());
                            tmp2 = cross(x4.getCAR(), x2.getCAR());
                            if (dot(tmp1, tmp2) > 0.0) {
                                I0 = I; J0 = J;
                                from0 = from; to0 = EastBnd; x = &x4;
                                I = I+1; if (I == numLon) I = 0;
                                from = WestBnd;
                                goto calc_overlap_area;
                            }
                        }
                    }
                }
                // northern boundary
                if ((from != NorthBnd && J > 0) || edgePointer != edgePointer0) {
                    Sphere::calcIntersectLon(x1, x2, latBnd1, x3, x4);
                    if (fabs(x3.getLat()-latBnd1) < EPS &&
                        Sphere::is_lon_between(lonBnd1, lonBnd2, x3.getLon())) {
                        if (dot(x1.getCAR(), x3.getCAR()) > 0.0) {
                            tmp1 = cross(x1.getCAR(), x3.getCAR());
                            tmp2 = cross(x3.getCAR(), x2.getCAR());
                            if (dot(tmp1, tmp2) > 0.0) {
                                I0 = I; J0 = J;
                                from0 = from; to0 = NorthBnd; x = &x3;
                                J = J-1;
                                from = SouthBnd;
                                goto calc_overlap_area;
                            }
                        }
                    }
                    if (fabs(x4.getLat()-latBnd1) < EPS &&
                        Sphere::is_lon_between(lonBnd1, lonBnd2, x4.getLon())) {
                        if (dot(x1.getCAR(), x4.getCAR()) > 0.0) {
                            tmp1 = cross(x1.getCAR(), x4.getCAR());
                            tmp2 = cross(x4.getCAR(), x2.getCAR());
                            if (dot(tmp1, tmp2) > 0.0) {
                                I0 = I; J0 = J;
                                from0 = from; to0 = NorthBnd; x = &x4;
                                J = J-1;
                                from = SouthBnd;
                                goto calc_overlap_area;
                            }
                        }
                    }
                }
                // southern boundary
                if ((from != SouthBnd && J < numLat) || edgePointer != edgePointer0) {
                    Sphere::calcIntersectLon(x1, x2, latBnd2, x3, x4);
                    if (fabs(x3.getLat()-latBnd2) < EPS &&
                        Sphere::is_lon_between(lonBnd1, lonBnd2, x3.getLon())) {
                        if (dot(x1.getCAR(), x3.getCAR()) > 0.0) {
                            tmp1 = cross(x1.getCAR(), x3.getCAR());
                            tmp2 = cross(x3.getCAR(), x2.getCAR());
                            if (dot(tmp1, tmp2) > 0.0) {
                                I0 = I; J0 = J;
                                from0 = from; to0 = SouthBnd; x = &x3;
                                J = J+1;
                                from = NorthBnd;
                                goto calc_overlap_area;
                            }
                        }
                    }
                    if (fabs(x4.getLat()-latBnd2) < EPS &&
                        Sphere::is_lon_between(lonBnd1, lonBnd2, x4.getLon())) {
                        if (dot(x1.getCAR(), x4.getCAR()) > 0.0) {
                            tmp1 = cross(x1.getCAR(), x4.getCAR());
                            tmp2 = cross(x4.getCAR(), x2.getCAR());
                            if (dot(tmp1, tmp2) > 0.0) {
                                I0 = I; J0 = J;
                                from0 = from; to0 = SouthBnd; x = &x4;
                                J = J+1;
                                from = NorthBnd;
                                goto calc_overlap_area;
                            }
                        }
                    }
                }
                // -------------------------------------------------------------
                // numerical tolerance
                // Note: When the polygon edge (great circle arc) is so close to
                //       the cell edge (lon/lat line) that numerical calculation
                //       of intersection may fail, therefore we need add some
                //       numerical tolerance.
                double smallAngleDistance;
                smallAngleDistance = (latBnd1-latBnd2)*0.1;
                if (!isForked) {
                    I0 = I; J0 = J; from0 = from;
                    if (fabs(x0.getLon()-lonBnd1) < smallAngleDistance &&
                        fabs(x0.getLat()-latBnd1) < smallAngleDistance) {
                        if (from == WestBnd) {
                            x3.set(lonBnd2, latBnd1);
                            to0 = EastBnd;
                            I = I+1; if (I == numLon) I = 0;
                        } else if (from == NorthBnd) {
                            x3.set(lonBnd1, latBnd2);
                            to0 = SouthBnd;
                            J = J+1;
                        }
                    } else if (fabs(x0.getLon()-lonBnd1) < smallAngleDistance &&
                               fabs(x0.getLat()-latBnd2) < smallAngleDistance) {
                        if (from == WestBnd) {
                            x3.set(lonBnd2, latBnd2);
                            to0 = EastBnd;
                            I = I+1; if (I == numLon) I = 0;
                        } else if (from == SouthBnd) {
                            x3.set(lonBnd1, latBnd1);
                            to0 = NorthBnd;
                            J = J-1;
                        }
                    } else if (fabs(x0.getLon()-lonBnd2) < smallAngleDistance &&
                               fabs(x0.getLat()-latBnd1) < smallAngleDistance) {
                        if (from == EastBnd) {
                            x3.set(lonBnd1, latBnd1);
                            to0 = WestBnd;
                            I = I-1; if (I == -1) I = numLon-1;
                        } else if (from == NorthBnd) {
                            x3.set(lonBnd2, latBnd2);
                            to0 = SouthBnd;
                            J = J+1;
                        }
                    } else if (fabs(x0.getLon()-lonBnd2) < smallAngleDistance &&
                               fabs(x0.getLat()-latBnd2) < smallAngleDistance) {
                        if (from == EastBnd) {
                            x3.set(lonBnd1, latBnd2);
                            to0 = WestBnd;
                            I = I-1; if (I == -1) I = numLon-1;
                        } else if (from == SouthBnd) {
                            x3.set(lonBnd2, latBnd1);
                            to0 = NorthBnd;
                            J = J-1;
                        }
                    } else
                        REPORT_ERROR("Can not find a path!");
                    x = &x3; isForked = true;
                    goto calc_overlap_area;
                } else {
                    if (from == EastBnd || from == WestBnd)
                        if (x0.getLat() == latBnd1)
                            J = J-1;
                        else if (x0.getLat() == latBnd2)
                            J = J+1;
                    else if (from == NorthBnd || from == SouthBnd)
                        if (x0.getLon() == lonBnd1) {
                            I = I-1; if (I == -1) I = numLon-1;
                        } else if (x0.getLon() == lonBnd2) {
                            I = I+1; if (I == numLon) I = 0;
                        }
                    isForked = false;
                    continue;
                }
                REPORT_ERROR("Can not find a path!");
            calc_overlap_area:
#ifdef DEBUG_INTERSECTION
                file << setw(5) << 1;
                file << setw(30) << setprecision(15) << x->getLon();
                file << setw(30) << setprecision(15) << x->getLat() << endl;
#endif
                if (edgePointer0 != NULL) {
                    double area = calcOverlapArea(I0, J0, from0, to0, bndDiff,
                                                  lonBnd1, lonBnd2,
                                                  latBnd1, latBnd2,
                                                  x0, edgePointer0,
                                                  *x, edgePointer);
                    recordOverlapArea(mesh.area(I0, J0), I0, J0,
                                      from0, to0, bndDiff,
                                      polygon, area, totalArea, overlapAreas);
                }
                // record the starting edge and intersection
                if (edgePointer00 == NULL) {
                    x00 = *x; edgePointer00 = edgePointer; to00 = to0;
                }
                // record the previous edge and intersection
                x0 = *x; edgePointer0 = edgePointer;
#ifdef DEBUG
                counter++;
#endif
            }
            edgePointer = edgePointer->next;
        }
#ifdef DEBUG_INTERSECTION
        file.close();
#endif
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
        // check the area difference
        diffArea = fabs(totalArea-realArea)/realArea;
        // ---------------------------------------------------------------------
        // handle the cells that are fully covered by the polygon
        if (diffArea > areaDiffThreshold) {
            // 1. get the cell range in which the cells should be checked
            int numCellLon = static_cast<int>(bndCellIdx.size()), numCellLat = 0;
            // reference index
            J0 = -1; J1 = -1;
            // Note: The following complicated codes are due to the zonal periodic
            //       boundary condition, please forgive me!
            map<int, list<int> >::iterator itIdxIJ1 = bndCellIdx.begin();
            map<int, list<int> >::iterator itIdxIJ2 = bndCellIdx.end();
            map<int, list<int> >::iterator itIdxIJ3 = bndCellIdx.end();
            map<int, list<int> >::iterator itIdxIJ4 = bndCellIdx.end();
            map<int, list<int> >::iterator itIdxIJ  = bndCellIdx.begin();
            map<int, list<int> >::iterator itIdxIJ0 = bndCellIdx.begin();
            for (++itIdxIJ0; itIdxIJ != bndCellIdx.end(); ++itIdxIJ, ++itIdxIJ0) {
                if (itIdxIJ0 != bndCellIdx.end() &&
                    (*itIdxIJ).first+1 != (*itIdxIJ0).first) {
                    itIdxIJ2 = itIdxIJ0;
                    itIdxIJ3 = itIdxIJ0;
                }
                (*itIdxIJ).second.sort();
                (*itIdxIJ).second.unique();
                if (J0 == -1)
                    J0 = (*itIdxIJ).second.front();
                else if (J0 > (*itIdxIJ).second.front())
                    J0 = (*itIdxIJ).second.front();
                if (J1 == -1)
                    J1 = (*itIdxIJ).second.back();
                else if (J1 < (*itIdxIJ).second.back())
                    J1 = (*itIdxIJ).second.back();
            }
            numCellLat = J1-J0+1;
            // 3. set the first guess cover mask
            Array<int, 2> coverMask(numCellLon, numCellLat);
            coverMask = 0;
            int idxI[numCellLon], idxJ[numCellLat];
            int i, j;
            // second part if the polygon crosses lon = 0 meridinal line
            for (i = 0, itIdxIJ = itIdxIJ3; itIdxIJ != itIdxIJ4; ++itIdxIJ, ++i) {
                idxI[i] = (*itIdxIJ).first;
                j = (*itIdxIJ).second.front()-J0;
                list<int>::const_iterator itIdxJ1 = (*itIdxIJ).second.begin();
                idxJ[j] = *itIdxJ1;
                coverMask(i, j++) = 1;
                list<int>::const_iterator itIdxJ2 = itIdxJ1;
                for (++itIdxJ1; itIdxJ1 != (*itIdxIJ).second.end(); ++itIdxJ1) {
                    if (*itIdxJ1-*itIdxJ2 != 1) {
                        for (J = *itIdxJ2+1; J < *itIdxJ1; ++J) {
                            idxJ[j] = J;
                            coverMask(i, j++) = -1;
                        }
                        idxJ[j] = J;
                        coverMask(i, j++) = 1;
                    } else {
                        idxJ[j] = *itIdxJ1;
                        coverMask(i, j++) = 1;
                    }
                    itIdxJ2++;
                }
            }
            // first part
            for (itIdxIJ = itIdxIJ1; itIdxIJ != itIdxIJ2; ++itIdxIJ, ++i) {
                idxI[i] = (*itIdxIJ).first;
                j = (*itIdxIJ).second.front()-J0;
                list<int>::const_iterator itIdxJ1 = (*itIdxIJ).second.begin();
                idxJ[j] = *itIdxJ1;
                coverMask(i, j++) = 1;
                list<int>::const_iterator itIdxJ2 = itIdxJ1;
                for (++itIdxJ1; itIdxJ1 != (*itIdxIJ).second.end(); ++itIdxJ1) {
                    if (*itIdxJ1-*itIdxJ2 != 1) {
                        for (J = *itIdxJ2+1; J < *itIdxJ1; ++J) {
                            idxJ[j] = J;
                            coverMask(i, j++) = -1;
                        }
                        idxJ[j] = J;
                        coverMask(i, j++) = 1;
                    } else {
                        idxJ[j] = *itIdxJ1;
                        coverMask(i, j++) = 1;
                    }
                    itIdxJ2++;
                }
            }
            // 4. set the cell that has been fully covered
#ifdef DEBUG
            if (debug) {
                for (i = 0; i < numCellLon; ++i)
                    cout << setw(5) << idxI[i];
                cout << endl;
                for (j = 0; j < numCellLat; ++j)
                    cout << idxJ[j] << endl;
                for (int jj = 0; jj < numCellLat; ++jj) {
                    for (int ii = 0; ii < numCellLon; ++ii)
                        cout << setw(3) << coverMask(ii, jj);
                    cout << endl;
                }
            }
#endif
            for (i = 0; i < numCellLon; ++i)
                for (j = 0; j < numCellLat; ++j) {
                    isCellCovered(numCellLon, numCellLat, coverMask, -1, -1, i, j);
#ifdef DEBUG
                    if (debug) {
                        cout << i << ", " << j << ":" << endl;
                        for (int jj = 0; jj < numCellLat; ++jj) {
                            for (int ii = 0; ii < numCellLon; ++ii) {
                                if (ii == i && jj == j)
                                    cout << " *" << setw(1) << coverMask(ii, jj);
                                else
                                    cout << setw(3) << coverMask(ii, jj);
                            }
                            cout << endl;
                        }
                    }
#endif
                }
            // 5. add the fully covered cells
            for (i = 0; i < numCellLon; ++i)
                for (j = 0; j < numCellLat; ++j)
                    if (coverMask(i, j) == 2)
                        recordOverlapArea(mesh.area(idxI[i], idxJ[j]),
                                          idxI[i], idxJ[j], polygon,
                                          mesh.area(idxI[i], idxJ[j]),
                                          totalArea, overlapAreas);
        }
        // ---------------------------------------------------------------------
        // check the area difference
        diffArea = fabs(totalArea-realArea)/realArea;
        // ---------------------------------------------------------------------
        // check if pole has been included
        if (diffArea > areaDiffThreshold) {
            // Note: Here we assume that if the boundary cells cover the whole
            //       zonal range, then the pole has been included
            if (bndCellIdx.size() == numLon) {
                Vertex *v = polygon->edgePointers.front()->getEndPoint(FirstPoint);
                Location::Pole checkPole = v->getCoordinate().getLat() > 0.0 ?
                Location::NorthPole : Location::SouthPole;
                map<int, list<int> >::const_iterator it1 = bndCellIdx.begin();
                for (; it1 != bndCellIdx.end(); ++it1) {
                    int i = (*it1).first;
                    switch (checkPole) {
                        case Location::NorthPole:
                            for (int j = 0; j < (*it1).second.front(); ++j)
                                recordOverlapArea(mesh.area(i, j), i, j, polygon,
                                                  mesh.area(i, j),
                                                  totalArea, overlapAreas);
                            break;
                        case Location::SouthPole:
                            for (int j = (*it1).second.back()+1; j < numLat; ++j)
                                recordOverlapArea(mesh.area(i, j), i, j, polygon,
                                                  mesh.area(i, j),
                                                  totalArea, overlapAreas);
                            break;
                        default:
                            break;
                    }
                }
                diffArea = fabs(totalArea-realArea)/realArea;
            }
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

    // reset tracer mass
    Polygon *polygon = tracerManager.polygonManager.polygons.front();
    for (int i = 0; i < tracerManager.polygonManager.polygons.size(); ++i) {
        polygon->tracers[tracerId].mass = 0.0;
        polygon = polygon->next;
    }

    double totalCellMass = 0.0, totalPolygonMass = 0.0;
    // accumulate tracer mass
    for (int i = 0; i < overlapAreaList.extent(0); ++i)
        for (int j = 0; j < overlapAreaList.extent(1); ++j) {
            double totalArea = 0.0;
            double cellMass = q.values(i, j).getNew()*mesh.area(i, j);
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
                (*itOa).polygon->tracers[tracerId].mass += cellMass*weight;
            }
        }

    // calculate tracer density
    polygon = tracerManager.polygonManager.polygons.front();
    for (int i = 0; i < tracerManager.polygonManager.polygons.size(); ++i) {
        polygon->tracers[tracerId].density
        = polygon->tracers[tracerId].mass/polygon->getArea(NewTimeLevel);
        totalPolygonMass += polygon->tracers[tracerId].mass;
        polygon = polygon->next;
    }

    cout << "Total cell mass is    " << setprecision(20) << totalCellMass << endl;
    cout << "Total polygon mass is " << setprecision(20) << totalPolygonMass << endl;
    double massError = totalCellMass-totalPolygonMass;
    cout << "Mass error is " << massError << endl;
    if (fabs(massError) > 1.0e-10)
        REPORT_ERROR("Mass error is too large!");
}

void MeshAdaptor::remap(const string &tracerName, TracerManager &tracerManager)
{
    NOTICE("MeshAdaptor::remap", "Remapping "+tracerName+" onto mesh ...");
    int tracerId = tracerManager.getTracerId(tracerName);
    Field &q = tracerManager.getTracerDensityField(tracerId);
    const RLLMesh &mesh = q.getMesh(Field::Bound);

    double totalCellMass = 0.0, totalPolygonMass = 0.0;
    for (int i = 0; i < overlapAreaList.extent(0); ++i)
        for (int j = 0; j < overlapAreaList.extent(1); ++j) {
            q.values(i, j, 0) = 0.0;
            list<OverlapArea>::const_iterator itOa;
            for (itOa = overlapAreaList(i, j, 0).begin();
                 itOa != overlapAreaList(i, j, 0).end(); ++itOa) {
                double weight = (*itOa).area/(*itOa).totalArea;
                q.values(i, j, 0) += (*itOa).polygon->tracers[tracerId].mass*weight;
            }
            totalCellMass += q.values(i, j, 0).getNew();
            q.values(i, j, 0) /= mesh.area(i, j);
        }

    Polygon *polygon = tracerManager.polygonManager.polygons.front();
    for (int i = 0; i < tracerManager.polygonManager.polygons.size(); ++i) {
        totalPolygonMass += polygon->tracers[tracerId].mass;
        polygon = polygon->next;
    }
    cout << "Total cell mass is    " << setprecision(20) << totalCellMass << endl;
    cout << "Total polygon mass is " << setprecision(20) << totalPolygonMass << endl;
    double massError = totalCellMass-totalPolygonMass;
    cout << "Mass error is " << massError << endl;
    if (fabs(massError) > 1.0e-10)
        REPORT_ERROR("Mass error is too large!");
}
