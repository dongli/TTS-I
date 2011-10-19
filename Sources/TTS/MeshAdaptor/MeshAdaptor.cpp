#include "MeshAdaptor.h"
#include "PolygonManager.h"
#include "MeshManager.h"
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

    overlapPolygons.resize(pointCounter.points.shape());
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
        //       of the edge from them, so use a test point to dig this out.
        // x4 is the coordinate of the test point
        Coordinate x3, x4;
        x3 = edgePointer0->getEndPoint(SecondPoint)->getCoordinate(NewTimeLevel);
        if (from == EastBnd || from == WestBnd)
            x4.set(x0.getLon(), (x0.getLat()+x1.getLat())*0.5);
        else if (from == NorthBnd || from == SouthBnd)
            x4.set((x0.getLon()+x1.getLon())*0.5, x0.getLat());
        if (Sphere::orient(x0, x3, x4) == OrientRight) bndDiff = 4;
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

    // ---------------------------------------------------------------------
    // get the number of polygon edges
    EdgePointer *edgePointer = edgePointer0;
    while (edgePointer != edgePointer1) {
        numPolygonEdge++;
        edgePointer = edgePointer->next;
    }

    // ---------------------------------------------------------------------
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
        normVectors[0] = norm_cross(x[0].getCAR(), x1.getCAR());
        for (i = 1; i < numCellEdge-1; ++i)
            normVectors[i] = norm_cross(x[i].getCAR(), x[i-1].getCAR());
        normVectors[i] = norm_cross(x0.getCAR(), x[i-1].getCAR());
    } else
        normVectors[0] = norm_cross(x0.getCAR(), x1.getCAR());

    // ---------------------------------------------------------------------
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

    // ---------------------------------------------------------------------
    // calculate the area of overlapping polygon (assuming all great circle
    // arc edges)
    excess = 0.0;
    for (i = 0; i < numEdge; ++i)
        excess += angles[i];
    excess -= (numEdge-2)*PI;
    area = excess*Sphere::radius2;

    // ---------------------------------------------------------------------
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
                                    double area, double &totalArea)
{
    list<OverlapPolygon> &ops = overlapPolygons(I, J, 0);
    // check multiply entried cell
    list<OverlapPolygon>::iterator it = ops.begin();
    for (; it != ops.end(); ++it) {
        if ((*it).polygon == polygon) {
            totalArea -= (*it).area;
            double testArea = (*it).area+area-cellArea;
            if (testArea > 0.0)
                (*it).area = testArea;
            else
                (*it).area += area;
            totalArea += (*it).area;
            (*it).from[(*it).numPart] = from;
            (*it).to[(*it).numPart] = to;
            (*it).bndDiff[(*it).numPart] = bndDiff;
            if (++(*it).numPart > 10)
                REPORT_ERROR("numPart has exceeded 10!");
            return;
        }
    }
    OverlapPolygon op;
    op.polygon = polygon;
    op.area = area;
    op.from[0] = from;
    op.to[0] = to;
    op.bndDiff[0] = bndDiff;
    op.numPart = 1;
    ops.push_back(op);
    totalArea += area;
}

void MeshAdaptor::recordOverlapArea(double cellArea, int I, int J,
                                    Polygon *polygon,
                                    double area, double &totalArea)
{
    list<OverlapPolygon> &ops = overlapPolygons(I, J, 0);
    // check multiply entried cell
    list<OverlapPolygon>::iterator it = ops.begin();
    for (; it != ops.end(); ++it) {
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
    OverlapPolygon op;
    op.polygon = polygon;
    op.area = area;
    ops.push_back(op);
    totalArea += area;
}

#define DEBUG_INTERSECTION

void MeshAdaptor::adapt(const PolygonManager &polygonManager,
                        const MeshManager &meshManager)
{
    NOTICE("MeshAdaptor::adapt", "running ...");
    const PointCounter &pointCounter = meshManager.pointCounter;

    int numLon = pointCounter.counters.extent(0);
    int numLat = pointCounter.counters.extent(1);

    double totalArea, diffArea, maxDiffArea = 0.0;
    map<int, list<int> > bndCellIdx;

    // reset
    for (int i = 0; i < overlapPolygons.extent(0); ++i)
        for (int j = 0; j < overlapPolygons.extent(1); ++j)
            overlapPolygons(i, j, 0).clear();

    // calculate the overlap area between polygon and mesh
    Polygon *polygon = polygonManager.polygons.front();
    for (int m = 0; m < polygonManager.polygons.size(); ++m) {
        //if (TimeManager::getSteps() == 20 && polygon->getID() == 162)
        if (TimeManager::getSteps() == 28 && polygon->getID() == 1821)
            REPORT_DEBUG;
        // record the previous edge and intersection
        EdgePointer *edgePointer0 = NULL; Coordinate x0;
        // record the starting edge and intersection
        EdgePointer *edgePointer00 = NULL; Coordinate x00;
        // record the coming and going boundary
        Bnd from0, to0, to00, from = NullBnd;
        // record the previous cell index
        int I0, J0;
        // reset
        totalArea = 0.0;
        bndCellIdx.clear();
        // internal variables
        int I, J, bndDiff;
        double lonBnd1, lonBnd2, latBnd1, latBnd2;
        Coordinate *x;
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
            int I1 = vertex1->getLocation().i[4];
            int J1 = vertex1->getLocation().j[4];
            int I2 = vertex2->getLocation().i[4];
            int J2 = vertex2->getLocation().j[4];
            // start from the cell where the first point is at
            I = I1, J = J1, I0 = I1, J0 = J1;
            while (true) {
                lonBnd1 = pointCounter.lonBnds(I);
                lonBnd2 = pointCounter.lonBnds(I+1);
                latBnd1 = pointCounter.latBnds(J);
                latBnd2 = pointCounter.latBnds(J+1);
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
                    if (x3.getLat() >= latBnd2 && x3.getLat() < latBnd1) {
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
                    if (x4.getLat() >= latBnd2 && x4.getLat() < latBnd1) {
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
                    if (x3.getLat() >= latBnd2 && x3.getLat() < latBnd1) {
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
                    if (x4.getLat() >= latBnd2 && x4.getLat() < latBnd1) {
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
                    if (Sphere::is_lon_between(lonBnd1, lonBnd2, x3.getLon())) {
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
                    if (Sphere::is_lon_between(lonBnd1, lonBnd2, x4.getLon())) {
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
                    if (Sphere::is_lon_between(lonBnd1, lonBnd2, x3.getLon())) {
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
                    if (Sphere::is_lon_between(lonBnd1, lonBnd2, x4.getLon())) {
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
                    recordOverlapArea(pointCounter.cellAreas(I0, J0),
                                      I0, J0, from0, to0, bndDiff,
                                      polygon, area, totalArea);
                }
                // record the starting edge and intersection
                if (edgePointer00 == NULL) {
                    x00 = *x; edgePointer00 = edgePointer; to00 = to0;
                }
                // record the previous edge and intersection
                x0 = *x; edgePointer0 = edgePointer;
            }
            edgePointer = edgePointer->next;
        }
#ifdef DEBUG_INTERSECTION
        file.close();
#endif
#ifdef DEBUG
        assert(edgePointer00 != NULL);
#endif
        double area = calcOverlapArea(I, J, from, to00, bndDiff,
                                      lonBnd1, lonBnd2, latBnd1, latBnd2,
                                      x0, edgePointer0, x00, edgePointer00);
        recordOverlapArea(pointCounter.cellAreas(I, J), I, J,
                          from, to00, bndDiff, polygon, area, totalArea);
        // ---------------------------------------------------------------------
        // handle the cells that are fully covered by the polygon
        map<int, list<int> >::iterator itIdxIJ = bndCellIdx.begin();
        for (; itIdxIJ != bndCellIdx.end(); ++itIdxIJ) {
            int i = (*itIdxIJ).first;
            (*itIdxIJ).second.sort();
            (*itIdxIJ).second.unique();
            list<int>::const_iterator itIdxJ1 = (*itIdxIJ).second.begin();
            list<int>::const_iterator itIdxJ2 = itIdxJ1;
            for (++itIdxJ1; itIdxJ1 != (*itIdxIJ).second.end(); ++itIdxJ1) {
                if (*itIdxJ1-*itIdxJ2 != 1) {
                    // check for concave polygon
                    list<OverlapPolygon>::const_iterator itOp = 
                    overlapPolygons(i, *itIdxJ2, 0).begin();
                    for (; itOp != overlapPolygons(i, *itIdxJ2, 0).end(); ++itOp)
                        if ((*itOp).polygon == polygon)
                            break;
                    bool isConcave = false;
                    for (int k = 0; k < (*itOp).numPart; ++k) {
                        Bnd from = (*itOp).from[k], to = (*itOp).to[k];
                        int bndDiff = (*itOp).bndDiff[k];
                        if (from == NorthBnd) {
                            if (to == NorthBnd || to == WestBnd)
                                continue;
                        } else if (from == EastBnd) {
                            if (to == NorthBnd || to == WestBnd)
                                continue;
                            if (to == EastBnd && bndDiff == 4)
                                continue;
                        } else if (from == WestBnd) {
                            if (to == WestBnd && bndDiff == 4)
                                continue;
                        }
                        isConcave = true;
                        break;
                    }
                    if (isConcave) continue;
                    for (int j = *itIdxJ2+1; j < *itIdxJ1; ++j) {
                        recordOverlapArea(pointCounter.cellAreas(i, j), i, j,
                                          polygon, pointCounter.cellAreas(i, j),
                                          totalArea);
                    }
                }
                itIdxJ2++;
            }
        }
        // ---------------------------------------------------------------------
        // check the area difference
        diffArea = fabs(totalArea-polygon->getArea(NewTimeLevel));
        // ---------------------------------------------------------------------
        // check if pole has been included
        if (diffArea > 1.0e-12) {
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
                            for (int j = 0; j < (*it1).second.front(); ++j) {
                                recordOverlapArea(pointCounter.cellAreas(i, j),
                                                  i, j, polygon,
                                                  pointCounter.cellAreas(i, j),
                                                  totalArea);
                            }
                            break;
                        case Location::SouthPole:
                            for (int j = (*it1).second.back()+1; j < numLat; ++j) {
                                recordOverlapArea(pointCounter.cellAreas(i, j),
                                                  i, j, polygon,
                                                  pointCounter.cellAreas(i, j),
                                                  totalArea);
                            }
                            break;
                        default:
                            break;
                    }
                }
                diffArea = fabs(totalArea-polygon->getArea(NewTimeLevel));
            }
        }
        if (diffArea > 1.0e-12) {
            ostringstream message;
            message << "Failed to calculate overlap area for polygon ";
            message << polygon->getID() << "!" << endl;
            REPORT_ERROR(message.str());
        }
        maxDiffArea = fmax(maxDiffArea, diffArea);
        polygon = polygon->next;
    }
#ifdef DEBUG
    cout << "Maximum area difference: " << maxDiffArea << endl;
#endif
}
