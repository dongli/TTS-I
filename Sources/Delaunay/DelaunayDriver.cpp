#include "DelaunayDriver.h"
#include "RandomNumber.h"
#include "Constants.h"
#include "Sphere.h"
#include "PointTriangle.h"
#include "ReportMacros.h"
#include <netcdfcpp.h>
#include <fstream>
#include <iomanip>

using std::setw;
using std::ofstream;

static int ip1[3] = {1,2,0};
static int im1[3] = {2,0,1};

DelaunayDriver::DelaunayDriver()
{
    DVT = NULL;
    DT = NULL;
    obsoleteDT = new List<DelaunayTrianglePointer>(10, 10);
    obsoleteDT->setName("Obsolete Delaunay triangles");
    temporalDT = new List<DelaunayTrianglePointer>(10, 10);
    temporalDT->setName("Temporal Delaunay triangles");
    REPORT_ONLINE("DelaunayDriver")
}

DelaunayDriver::~DelaunayDriver()
{
    if (DVT != NULL) DVT->destroy(), delete DVT;
    if (DT != NULL) DT->destroy(), delete DT;
    obsoleteDT->destroy(), delete obsoleteDT;
    temporalDT->destroy(), delete temporalDT;
    REPORT_OFFLINE("DelaunayDriver")
}

void DelaunayDriver::linkPoint(const PointManager &pointManager)
{
    DVT = new List<DelaunayVertex>(pointManager.points.size());
    DVT->setName("Delaunay vertices");
    PointPointer *point = pointManager.points.front();
    for (int i = 0; i < pointManager.points.size(); ++i) {
        DVT->append();
        DVT->back()->point = point->ptr;
        point = point->next;
    }
}

void DelaunayDriver::construct()
{
    int idx[3];
    
    getThreeRandomIndices(idx);
    initDelaunayTriangle(idx);
    initPIT();
    insertRestPoints();
    DT->reindex();
    extractTopology();
    deleteFake();
}

void DelaunayDriver::calcircum()
{
    DelaunayTriangle *DT = this->DT->front();
    for (int i = 0; i < this->DT->size(); ++i) {
        DT->calcircum();
        DT = DT->next;
    }
}

void DelaunayDriver::getCircumcenter(double lon[], double lat[])
{
    DelaunayTriangle *DT = this->DT->front();
    for (int i = 0; i < this->DT->size(); ++i) {
        lon[i] = DT->circumcenter.getCoordinate().getLon();
        lat[i] = DT->circumcenter.getCoordinate().getLat();
        DT = DT->next;
    }
}

void DelaunayDriver::getThreeRandomIndices(int idx[])
{
    if (DVT->size() == 3) {
        idx[0] = 0; idx[1] = 1; idx[2] = 2;
        return;
    }
    bool success = false;
    while (!success) {
        for (int i = 0; i < 3; ++i)
            idx[i] = RandomNumber::getRandomNumber(0, DVT->size()-1);
        if (idx[0] != idx[1] && idx[1] != idx[2] && idx[2] != idx[0])
            success = true;
    }
}

void DelaunayDriver::initDelaunayTriangle(int threeIdx[])
{
    DT = new List<DelaunayTriangle>(1000, 100);
    DT->setName("Delaunay triangles");

    // 0. Use an array to uniformly access the three vertices
    DelaunayVertex *DVT[6];
    DVT[0] = this->DVT->at(threeIdx[0]);
    DVT[1] = this->DVT->at(threeIdx[1]);
    DVT[2] = this->DVT->at(threeIdx[2]);
    
    // 1. Set the three fake vertices that are antipodal to the corresponding
    //    vertices of the first three inserted ones
    Coordinate southPole, x;
    southPole.set(0.0, -PI05);
    for (int i = 0; i < 3; ++i) {
        DVT[3+i] = &(fake.DVT[i]);
        Sphere::inverseRotate(DVT[i]->point->getCoordinate(), x, southPole);
        fake.DVT[i].point->setCoordinate(x);
    }
    
    // 2. Create the first eight triangles and  Use an array 
    //    to uniformly access them
    DelaunayTriangle *DT[8];
    for (int i = 0; i < 8; ++i)
        this->DT->append(&DT[i]);
    
    // 3. Link the triangles with their adjacent triangles
    static int idx1[24] = {4,1,3, 5,2,0, 6,3,1, 7,0,2, 0,7,5, 1,4,6, 2,5,7, 3,6,4};
    int* idx = idx1;
    int j = -1;
    for (int i = 0; i < 8; ++i) {
        DT[i]->adjDT[0] = DT[idx[++j]];
        DT[i]->adjDT[1] = DT[idx[++j]];
        DT[i]->adjDT[2] = DT[idx[++j]];
    }
    
    // 4. Link triangles with their vertices
    int ret;
    ret = Sphere::orient(DVT[0]->point, DVT[1]->point, DVT[2]->point);
    static int idx2[24] = {0,1,2, 0,2,4, 0,4,5, 0,5,1, 3,2,1, 3,4,2, 3,5,4, 3,1,5};
    static int idx3[24] = {0,2,1, 0,1,5, 0,5,4, 0,4,2, 3,1,2, 3,5,1, 3,4,5, 3,2,4};
    if (ret == OrientLeft) {
        idx = idx2;
    } else if (ret == OrientRight) {
        idx = idx3;
    } else if (ret == OrientOn) {
        REPORT_ERROR("The first points are collinear.")
    }
    j = -1;
    for (int i = 0; i < 8; ++i) {
        DT[i]->DVT[0] = DVT[idx[++j]];
        DT[i]->DVT[1] = DVT[idx[++j]];
        DT[i]->DVT[2] = DVT[idx[++j]];
    }
    
    //  5. Link vertices with one of its incident triangles
    //     Note:  Any one is ok, the full list will be extracted at the end of
    //            the Delaunay triangulation
    DVT[0]->topology.incidentDT->append();
    DVT[0]->topology.incidentDT->front()->ptr = DT[0];
    DVT[1]->topology.incidentDT->append();
    DVT[1]->topology.incidentDT->front()->ptr = DT[0];
    DVT[2]->topology.incidentDT->append();
    DVT[2]->topology.incidentDT->front()->ptr = DT[0];
    DVT[3]->topology.incidentDT->append();
    DVT[3]->topology.incidentDT->front()->ptr = DT[6];
    DVT[4]->topology.incidentDT->append();
    DVT[4]->topology.incidentDT->front()->ptr = DT[6];
    DVT[5]->topology.incidentDT->append();
    DVT[5]->topology.incidentDT->front()->ptr = DT[6];
    
    DVT[0]->inserted = true;
    DVT[1]->inserted = true;
    DVT[2]->inserted = true;
}

void DelaunayDriver::initPIT()
{
    DelaunayVertex *DVT;
    DelaunayTriangle *DT;
    int i, j;
    
    // Loop for each point
    DVT = this->DVT->front();
    for (i = 0; i < this->DVT->size(); ++i) {
        if (DVT->inserted) {
            DVT = DVT->next;
            continue;
        }
        // Loop for each triangle
        DT = this->DT->front();
        for (j = 0; j < this->DT->size(); ++j) {
            int ret = Sphere::inTriangle(DT->DVT[0]->point,
                                         DT->DVT[1]->point,
                                         DT->DVT[2]->point,
                                         DVT->point);
            if (ret == 4) { // inside triangle
                PointTriangle::recordPoint(DVT, DT);
                break;
            } else if (ret == -4) { // outside triangle
                DT = DT->next;
                continue;
            } else if (ret > 0) {
                // Point is on some edge of a triangle
                --ret;
                DelaunayTriangle *adjDT = DT->adjDT[ret];
                PointTriangle::recordPoint(DVT, DT, adjDT, ret);
                break;
            } else if (ret < 0) {
                // Point coincides with a vertex of a triangle
                ret = -(ret+1);
                DelaunayVertex *DVT1 = DT->DVT[ret];
                if (DVT1->getID() < 0) {
                    // The vertex is fake
                    DVT1->topology.extract();
                    // Record the first incident triangle
                    DVT->topology.incidentDT->append();
                    DVT->topology.incidentDT->front()->ptr = 
                        DVT1->topology.incidentDT->front()->ptr;
                    // Replace the fake vertex with the real one
                    DelaunayTrianglePointer *DTptr;
                    DTptr = DVT1->topology.incidentDT->front();
                    for (int k = 0; k < DVT1->topology.incidentDT->size(); ++k) {
                        int l;
                        for (l = 0; l < 3; ++l)
                            if (DTptr->ptr->DVT[l] == DVT1) break;
                        DTptr->ptr->DVT[l] = DVT;
                        DTptr = DTptr->next;
                    }
                    DVT->inserted = true;
                    // Delete the fake vertex
                    for (int i = 0; i < fake.num; ++i)
                        if (DVT1 == &(fake.DVT[i])) {
                            fake.erase(i);
                            break;
                        }
                    break;
                } else {
                    REPORT_ERROR("Encounter coincided vertices.")
                }
            }
        }
        if (j == 8) {
            REPORT_ERROR("Point is not covered by any of the eight triangles.")
        }
        DVT = DVT->next;
    }
}

void DelaunayDriver::insertRestPoints()
{
    DelaunayVertex *DVT = this->DVT->front();
    for (int i = 0; i < this->DVT->size(); ++i) {
        if (DVT->inserted) {
            DVT = DVT->next;
            continue;
        }
        insertPoint(DVT);
        updatePIT();
        deleteObsoleteDT();
        deleteTemporalDT();
        DVT = DVT->next;
    } 
}

void DelaunayDriver::insertPoint(DelaunayVertex *point)
{
    if (point->pit.DT[1] == NULL) {
        DelaunayTriangle *oldDT = point->pit.DT[0];
        flip13(oldDT, point);
        obsoleteDT->append();
        obsoleteDT->back()->ptr = oldDT;
        PointTriangle::removePoint(point);
    } else {
        DelaunayTriangle *oldDT1 = point->pit.DT[0];
        DelaunayTriangle *oldDT2 = point->pit.DT[1];
        flip24(oldDT1, oldDT2, point);
        obsoleteDT->append();
        obsoleteDT->back()->ptr = oldDT1;
        obsoleteDT->append();
        obsoleteDT->back()->ptr = oldDT2;
        PointTriangle::removePoint(point);
    }
    point->inserted = true;
}

void DelaunayDriver::updatePIT()
{
    DelaunayTrianglePointer *DT = obsoleteDT->front();
    for (int i = 0; i < obsoleteDT->size(); ++i) {
        while (DT->ptr->tip.points->size() != 0) {
            if (!DT->ptr->tip.handover(DT->ptr->tip.points->front()->ptr)) {
                REPORT_ERROR("Point hasn't been handed over!")
            }
        }
        DT = DT->next;
    }
}

void DelaunayDriver::deleteObsoleteDT()
{
    DelaunayTrianglePointer *DT = obsoleteDT->front();
    for (int i = 0; i < obsoleteDT->size(); ++i) {
        this->DT->remove(DT->ptr);
        DT = DT->next;
    }
    obsoleteDT->recycle();
}

void DelaunayDriver::deleteTemporalDT()
{
    DelaunayTrianglePointer *DT = temporalDT->front();
    for (int i = 0; i < temporalDT->size(); ++i) {
        this->DT->remove(DT->ptr);
        DT = DT->next;
    }
    temporalDT->recycle();
}

void DelaunayDriver::deleteDVT(DelaunayVertex *DVT)
{
    DelaunayTrianglePointer *DTptr = DVT->topology.incidentDT->front();
    DelaunayVertexPointer *DVTptr = DVT->topology.linkDVT->front();
    DelaunayTriangle *DT1, *DT2, *DT3;
    DelaunayVertex *DVT1, *DVT2, *DVT3;
    int i, j, k, ret;
    
    while (DVT->topology.incidentDT->size() > 3) {
        DVT1 = DVTptr->ptr;
        DVT2 = DVTptr->next->ptr;
        DVT3 = DVTptr->next->next->ptr;
        // Check 1: Is potential triangle convex?
        ret = Sphere::orient(DVT1->point, DVT2->point, DVT3->point);
        if (ret == OrientRight) {
            DTptr = DTptr->next;
            DVTptr = DVTptr->next;
            continue; // NOT PASS
        }
        // Check 2: Does potential triangle include DVT?
        ret = Sphere::orient(DVT3->point, DVT1->point, DVT->point);
        if (ret == OrientLeft) {
            DTptr = DTptr->next;
            DVTptr = DVTptr->next;
            continue; // NOT PASS
        }
        // Check 3: Does potential triangle satisfy empty-circumcircle rule?
        bool empty = true;
        DelaunayVertexPointer *restDVT = DVTptr->next->next->next;
        for (i = 0; i < DVT->topology.incidentDT->size()-3; ++i) {
            ret = Sphere::inCircle(DVT1->point, DVT2->point, DVT3->point,
                                   restDVT->ptr->point);
            if (ret == InsideCircle) {
                empty = false;
                break;
            } else if (ret == OnCircle) {
                REPORT_ERROR("Encounter cocircular vertices.")
            }
            restDVT = restDVT->next;
        }
        if (!empty) {
            DTptr = DTptr->next;
            DVTptr = DVTptr->next;
            continue; // NOT PASS
        }
        // So far, the potential trianlge is a real Delaunay triangle
        DT1 = DTptr->ptr;
        DT2 = DTptr->next->ptr;
        for (i = 0; i < 3; ++i)
            if (DT1->DVT[i] == DVT)
                break;
        for (j = 0; j < 3; ++j)
            if (DT2->DVT[j] == DVT)
                break;
        DelaunayTriangle *newDT1, *newDT2;
        int idxMap[4] = {im1[i],i,ip1[i],im1[j]};
        flip22(DT1, DT2, &newDT1, &newDT2, idxMap);
        DT->remove(DT1);
        DT->remove(DT2);
    }
    
    // Here, there are only three triangles
    DTptr = DVT->topology.incidentDT->front();
    DT1 = DTptr->ptr;
    DT2 = DTptr->next->ptr;
    DT3 = DTptr->next->next->ptr;
    DVTptr = DVT->topology.linkDVT->front();
    DVT1 = DVTptr->ptr;
    DVT2 = DVTptr->next->ptr;
    DVT3 = DVTptr->next->next->ptr;
    for (i = 0; i < 3; ++i)
        if (DT1->DVT[i] == DVT)
            break;
    for (j = 0; j < 3; ++j)
        if (DT2->DVT[j] == DVT)
            break;
    for (k = 0; k < 3; ++k)
        if (DT3->DVT[k] == DVT)
            break;
    DelaunayTriangle *newDT;
    int idxMap[5] = {ip1[i],im1[i],i,j,k};
    flip31(DT1, DT2, DT3, &newDT, idxMap);
    DT->remove(DT1);
    DT->remove(DT2);
    DT->remove(DT3);
}

void DelaunayDriver::deleteFake()
{
    for (int i = 0; i < fake.num; ++i)
        if (!fake.deleted[i]) {
            deleteDVT(&fake.DVT[i]);
            fake.erase(i);
        }
}

void DelaunayDriver::extractTopology()
{
    DelaunayVertex *DVT = this->DVT->front();
    for (int i = 0; i < this->DVT->size(); ++i) {
        DVT->topology.extract();
        DVT = DVT->next;
    }
    for (int i = 0; i < fake.num; ++i)
        if (!fake.deleted[i])
            fake.DVT[i].topology.extract();
}

void DelaunayDriver::validate(DelaunayTriangle *DT)
{
    DelaunayTriangle *oppositeDT;
    DelaunayVertex *oppositeDVT;
    int idx;
    
    // 1. Get opposite triangle
    oppositeDT = DT->adjDT[2];
    // 2. Get the opposite vertex
    for (idx = 0; idx < 3; ++idx) {
        if (oppositeDT->adjDT[idx] == DT) {
            oppositeDVT = oppositeDT->DVT[idx];
            break;
        }
    }
#ifdef DEBUG
    if (idx == 3) {
        REPORT_ERROR("No matched opposite Delaunay vertex.")
    }
#endif
    // 3. Check if the opposite vertex is in the circumcircle of triangle
    int ret = Sphere::inCircle(DT->DVT[0]->point, DT->DVT[1]->point,
                               DT->DVT[2]->point, oppositeDVT->point);
    if (ret == 2) { // outside circumcircle
        return; // The only exit!
    } else if (ret == 1) { // inside circumcircle
        DelaunayTriangle *newDT1, *newDT2;
        int idxMap[] = {0,1,2,idx};
        flip22(DT, oppositeDT, &newDT1, &newDT2, idxMap);
        // Recursively validate the two new triangles
        validate(newDT1);
        validate(newDT2);
        // Record the temporal triangle
        temporalDT->append();
        temporalDT->back()->ptr = DT;
        DT->subDT[0] = newDT1;
        DT->subDT[1] = newDT2;
        // Record the obsolete triangle
        obsoleteDT->append();
        obsoleteDT->back()->ptr = oppositeDT;
        oppositeDT->subDT[0] = newDT1;
        oppositeDT->subDT[1] = newDT2;
    } else {
#ifdef DEBUG
        //REPORT_WARNING("Encounter cocircular points.")
#endif
    }
}

void DelaunayDriver::flip13(DelaunayTriangle *oldDT, DelaunayVertex *point)
{
    // 0. Make short-hand
    DelaunayVertex *DVT[3];
    for (int i = 0; i < 3; ++i)
        DVT[i] = oldDT->DVT[i];
    DelaunayTriangle *adjDT[3];
    adjDT[0] = oldDT->adjDT[2];
    adjDT[1] = oldDT->adjDT[0];
    adjDT[2] = oldDT->adjDT[1];
    // 1. Subdivide the old triangle into three new ones
    DelaunayTriangle *newDT[3];
    for (int i = 0; i < 3; ++i)
        this->DT->append(&newDT[i]);
    // 2. Set up the topology of the three triangles
    // 2.1 New triangle 1
    newDT[0]->DVT[0] = DVT[0];
    newDT[0]->DVT[1] = DVT[1];
    newDT[0]->DVT[2] = point;
    newDT[0]->adjDT[0] = newDT[1];
    newDT[0]->adjDT[1] = newDT[2];
    newDT[0]->adjDT[2] = adjDT[0];
     // 2.2 New triangle 2
    newDT[1]->DVT[0] = DVT[1];
    newDT[1]->DVT[1] = DVT[2];
    newDT[1]->DVT[2] = point;
    newDT[1]->adjDT[0] = newDT[2];
    newDT[1]->adjDT[1] = newDT[0];
    newDT[1]->adjDT[2] = adjDT[1];
    // 2.3 New triangle 3
    newDT[2]->DVT[0] = DVT[2];
    newDT[2]->DVT[1] = DVT[0];
    newDT[2]->DVT[2] = point;
    newDT[2]->adjDT[0] = newDT[0];
    newDT[2]->adjDT[1] = newDT[1];
    newDT[2]->adjDT[2] = adjDT[2];
    // 3. Link the newly inserted point to one of its incident triangles
    point->topology.incidentDT->append();
    point->topology.incidentDT->back()->ptr = newDT[0];
    // 4. Make change to the vertices and adjacent triangles of the old triangle
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j)
            if (adjDT[i]->adjDT[j] == oldDT) {
                adjDT[i]->adjDT[j] = newDT[i];
                break;
            }
        if (DVT[i]->topology.incidentDT->front()->ptr == oldDT)
            DVT[i]->topology.incidentDT->front()->ptr = newDT[i];
    }
    // 5. Validate the new triangles
    for (int i = 0; i < 3; ++i)
        validate(newDT[i]);
    // 6. Record the new triangles as the subdivided triangles
    for (int i = 0; i < 3; ++i)
        oldDT->subDT[i] = newDT[i];
}

void DelaunayDriver::flip24(DelaunayTriangle *oldDT1, DelaunayTriangle *oldDT2,
                            DelaunayVertex *point)
{
    int idx;
    for (idx = 0; idx < 3; ++idx)
        if (oldDT2->adjDT[idx] == oldDT1) break;
    DelaunayVertex *DVT1, *DVT2, *DVT3, *DVT4;
    DVT1 = oldDT1->DVT[point->pit.edgeIdx];
    DVT2 = oldDT1->DVT[ip1[point->pit.edgeIdx]];
    DVT3 = oldDT1->DVT[im1[point->pit.edgeIdx]];
    DVT4 = oldDT2->DVT[idx];
    DelaunayTriangle *adjDT1, *adjDT2, *adjDT3, *adjDT4;
    adjDT1 = oldDT1->adjDT[ip1[point->pit.edgeIdx]];
    adjDT2 = oldDT1->adjDT[im1[point->pit.edgeIdx]];
    adjDT3 = oldDT2->adjDT[ip1[idx]];
    adjDT4 = oldDT2->adjDT[im1[idx]];
    // 1. Subdivide the two triangles into four new ones
    DelaunayTriangle *newDT[4];
    for (int i = 0; i < 4; ++i)
        this->DT->append(&newDT[i]);
    // 2. Set up the topology of the four triangles
    // 2.1 New triangle 1
    newDT[0]->DVT[0] = DVT3;
    newDT[0]->DVT[1] = DVT1;
    newDT[0]->DVT[2] = point;
    newDT[0]->adjDT[0] = newDT[1];
    newDT[0]->adjDT[1] = newDT[3];
    newDT[0]->adjDT[2] = adjDT1;
    // 2.2 New triangle 2
    newDT[1]->DVT[0] = DVT1;
    newDT[1]->DVT[1] = DVT2;
    newDT[1]->DVT[2] = point;
    newDT[1]->adjDT[0] = newDT[2];
    newDT[1]->adjDT[1] = newDT[0];
    newDT[1]->adjDT[2] = adjDT2;
    // 2.3 New triangle 3
    newDT[2]->DVT[0] = DVT2;
    newDT[2]->DVT[1] = DVT4;
    newDT[2]->DVT[2] = point;
    newDT[2]->adjDT[0] = newDT[3];
    newDT[2]->adjDT[1] = newDT[1];
    newDT[2]->adjDT[2] = adjDT3;
    // 2.4 New triangle 4
    newDT[3]->DVT[0] = DVT4;
    newDT[3]->DVT[1] = DVT3;
    newDT[3]->DVT[2] = point;
    newDT[3]->adjDT[0] = newDT[0];
    newDT[3]->adjDT[1] = newDT[2];
    newDT[3]->adjDT[2] = adjDT4;
    // 3. Link the newly inserted point to one of its incident triangles
    point->topology.incidentDT->append();
    point->topology.incidentDT->front()->ptr = newDT[0];
    // 4. Make change to the vertices and adjacent triangles of the old 
    //    triangle
    for (int i = 0; i < 3; ++i)
        if (adjDT1->adjDT[i] == oldDT1) {
            adjDT1->adjDT[i] = newDT[0];
            break;
        }
    for (int i = 0; i < 3; ++i)
        if (adjDT2->adjDT[i] == oldDT1) {
            adjDT2->adjDT[i] = newDT[1];
            break;
        }
    for (int i = 0; i < 3; ++i)
        if (adjDT3->adjDT[i] == oldDT2) {
            adjDT3->adjDT[i] = newDT[2];
            break;
        }
    for (int i = 0; i < 3; ++i)
        if (adjDT4->adjDT[i] == oldDT2) {
            adjDT4->adjDT[i] = newDT[3];
            break;
        }
    // Notice the order of newDT!!!
    if (DVT1->topology.incidentDT->front()->ptr == oldDT1)
        DVT1->topology.incidentDT->front()->ptr = newDT[1];
    if (DVT2->topology.incidentDT->front()->ptr == oldDT1 ||
        DVT2->topology.incidentDT->front()->ptr == oldDT2)
        DVT2->topology.incidentDT->front()->ptr = newDT[2];
    if (DVT3->topology.incidentDT->front()->ptr == oldDT1 ||
        DVT3->topology.incidentDT->front()->ptr == oldDT2)
        DVT3->topology.incidentDT->front()->ptr = newDT[3];
    if (DVT4->topology.incidentDT->front()->ptr == oldDT2)
        DVT4->topology.incidentDT->front()->ptr = newDT[0];
    // 5. Validate the four triangles
    for (int i = 0; i < 4; ++i)
        validate(newDT[i]);
    // 6. Record the new triangles as the subdivided triangles
    oldDT1->subDT[0] = newDT[0];
    oldDT1->subDT[1] = newDT[1];
    oldDT2->subDT[0] = newDT[2];
    oldDT2->subDT[1] = newDT[3];
}

void DelaunayDriver::flip22(DelaunayTriangle *oldDT1, DelaunayTriangle *oldDT2,
                            DelaunayTriangle **newDT1, DelaunayTriangle **newDT2,
                            int idxMap[])
{
    // 0. Make short-hand
    DelaunayVertex *DVT1, *DVT2, *DVT3, *DVT4;
    DVT1 = oldDT1->DVT[idxMap[0]];
    DVT2 = oldDT1->DVT[idxMap[1]];
    DVT3 = oldDT1->DVT[idxMap[2]];
    DVT4 = oldDT2->DVT[idxMap[3]];
    DelaunayTriangle *adjDT1, *adjDT2, *adjDT3, *adjDT4;
    adjDT1 = oldDT1->adjDT[idxMap[0]];
    adjDT2 = oldDT1->adjDT[idxMap[1]];
    adjDT3 = oldDT2->adjDT[ip1[idxMap[3]]];
    adjDT4 = oldDT2->adjDT[im1[idxMap[3]]];
    // 1. Create two new triangles and set up their topology
    DT->append(newDT1);
    DT->append(newDT2);
    (*newDT1)->DVT[0] = DVT1;
    (*newDT1)->DVT[1] = DVT4;
    (*newDT1)->DVT[2] = DVT3;
    (*newDT1)->adjDT[0] = *newDT2;
    (*newDT1)->adjDT[1] = adjDT2;
    (*newDT1)->adjDT[2] = adjDT3;
    (*newDT2)->DVT[0] = DVT4;
    (*newDT2)->DVT[1] = DVT2;
    (*newDT2)->DVT[2] = DVT3;
    (*newDT2)->adjDT[0] = adjDT1;
    (*newDT2)->adjDT[1] = *newDT1;
    (*newDT2)->adjDT[2] = adjDT4;
    // 2. Make change to the old triangles
    for (int i = 0; i < 3; ++i)
        if (adjDT1->adjDT[i] == oldDT1) {
            adjDT1->adjDT[i] = *newDT2;
            break;
        }
    for (int i = 0; i < 3; ++i)
        if (adjDT2->adjDT[i] == oldDT1) {
            adjDT2->adjDT[i] = *newDT1;
            break;
        }
    for (int i = 0; i < 3; ++i)
        if (adjDT3->adjDT[i] == oldDT2) {
            adjDT3->adjDT[i] = *newDT1;
            break;
        }
    for (int i = 0; i < 3; ++i)
        if (adjDT4->adjDT[i] == oldDT2) {
            adjDT4->adjDT[i] = *newDT2;
            break;
        }
    if (DVT1->topology.isComplete) {
        DVT1->topology.mergeIncidentDT(oldDT2, oldDT1, *newDT1);
        DVT2->topology.mergeIncidentDT(oldDT1, oldDT2, *newDT2);
        DVT3->topology.splitIncidentDT(oldDT1, *newDT1, *newDT2);
        DVT4->topology.splitIncidentDT(oldDT2, *newDT2, *newDT1);
        DVT1->topology.deleteLinkDVT(DVT2);
        DVT2->topology.deleteLinkDVT(DVT1);
        DVT3->topology.addLinkDVT(DVT1, DVT2, DVT4);
        DVT4->topology.addLinkDVT(DVT2, DVT1, DVT3);
    } else {
        DVT1->topology.incidentDT->front()->ptr = *newDT1;
        DVT2->topology.incidentDT->front()->ptr = *newDT2;
        DVT3->topology.incidentDT->front()->ptr = *newDT2;
        DVT4->topology.incidentDT->front()->ptr = *newDT1;
    }
}

void DelaunayDriver::flip31(DelaunayTriangle *oldDT1, DelaunayTriangle *oldDT2,
                            DelaunayTriangle *oldDT3, DelaunayTriangle **newDT,
                            int idxMap[])
{
    // 0. Make short-hand
    DelaunayVertex *DVT1, *DVT2, *DVT3, *DVT4;
    DVT1 = oldDT1->DVT[idxMap[0]];
    DVT2 = oldDT1->DVT[idxMap[1]];
    DVT3 = oldDT1->DVT[idxMap[2]];
    DVT4 = oldDT2->DVT[im1[idxMap[3]]];
    DelaunayTriangle *adjDT1, *adjDT2, *adjDT3;
    adjDT1 = oldDT1->adjDT[idxMap[2]];
    adjDT2 = oldDT2->adjDT[idxMap[3]];
    adjDT3 = oldDT3->adjDT[idxMap[4]];
    // 1. Create one new triangles and set up their topology
    DT->append(newDT);
    (*newDT)->DVT[0] = DVT1;
    (*newDT)->DVT[1] = DVT2;
    (*newDT)->DVT[2] = DVT4;
    (*newDT)->adjDT[0] = adjDT2;
    (*newDT)->adjDT[1] = adjDT3;
    (*newDT)->adjDT[2] = adjDT1;
    // 2. Make change to the old triangle
    for (int i = 0; i < 3; ++i)
        if (adjDT1->adjDT[i] == oldDT1) {
            adjDT1->adjDT[i] = *newDT;
            break;
        }
    for (int i = 0; i < 3; ++i)
        if (adjDT2->adjDT[i] == oldDT2) {
            adjDT2->adjDT[i] = *newDT;
            break;
        }
    for (int i = 0; i < 3; ++i)
        if (adjDT3->adjDT[i] == oldDT3) {
            adjDT3->adjDT[i] = *newDT;
            break;
        }
    DVT1->topology.mergeIncidentDT(oldDT1, oldDT3, *newDT);
    DVT2->topology.mergeIncidentDT(oldDT2, oldDT1, *newDT);
    DVT4->topology.mergeIncidentDT(oldDT3, oldDT2, *newDT);
    DVT1->topology.deleteLinkDVT(DVT3);
    DVT2->topology.deleteLinkDVT(DVT3);
    DVT4->topology.deleteLinkDVT(DVT3);
}

void DelaunayDriver::output(const string &fileName)
{
    outputNetCDF(fileName);
}

inline void DelaunayDriver::outputAscii(const string &fileName)
{
    ofstream file;
    file.open(string(fileName+".txt").c_str());
    file << "[Delaunay triangles]" << endl;
    file << "Number: " << this->DT->size() << endl;
    DelaunayTriangle *DT = this->DT->front();
    for (int i = 0; i < this->DT->size(); ++i) {
        file << "#" << setw(8) << DT->getID();
        for (int j = 0; j < 3; ++j)
            file << setw(10) << DT->DVT[j]->getID();
        file << endl;
        DT = DT->next;
    }
    file << "[Delaunay vertices]" << endl;
    file << "Number: " << this->DVT->size() << endl;
    DelaunayVertex *DVT = this->DVT->front();
    for (int i = 0; i < this->DVT->size(); ++i) {
        file << "#" << setw(8) << DVT->getID();
        file << DVT->point << endl;
        DVT = DVT->next;
    }
    int numFake = 0;
    for (int i = 0; i < fake.num; ++i)
        if (!fake.deleted[i]) ++numFake;
    file << "[Fake vertices]" << endl;
    file << "Number: " << numFake << endl;
    for (int i = 0; i < fake.num; ++i) {
        if (!fake.deleted[i]) {
            file << "#" << setw(8) << fake.DVT[i].getID();
            file << fake.DVT[i].point << endl;
        }
    }
    file.close();
}

inline void DelaunayDriver::outputNetCDF(const string &fileName)
{
    NcFile *file = new NcFile(string(fileName+".nc").c_str(), NcFile::Replace);
    
    // Output Delaunay vertices
    NcDim *numDVTDim = file->add_dim("numDVT", this->DVT->size());
    NcVar *lonDVTVar = file->add_var("lonDVT", ncDouble, numDVTDim);
    NcVar *latDVTVar = file->add_var("latDVT", ncDouble, numDVTDim);
    double lonDVT[this->DVT->size()], latDVT[this->DVT->size()];
    DelaunayVertex *DVT = this->DVT->front();
    for (int i = 0; i < this->DVT->size(); ++i) {
        lonDVT[i] = DVT->point->getCoordinate().getLon()*Rad2Deg;
        latDVT[i] = DVT->point->getCoordinate().getLat()*Rad2Deg;
        DVT = DVT->next;
    }
    lonDVTVar->put(lonDVT, this->DVT->size());
    latDVTVar->put(latDVT, this->DVT->size());
    
    // Output the remaining fake vertices
    bool hasFakeDVT = false;
    for (int i = 0; i < fake.num; ++i)
        if (!fake.deleted[i]) {
            hasFakeDVT = true;
            break;
        }
    if (hasFakeDVT) {
        NcDim *numFakeDVTDim = file->add_dim("numFakeDVT", fake.num);
        NcVar *lonFakeDVTVar = file->add_var("lonFakeDVT", ncDouble, numFakeDVTDim);
        NcVar *latFakeDVTVar = file->add_var("latFakeDVT", ncDouble, numFakeDVTDim);
        double lonFakeDVT[fake.num], latFakeDVT[fake.num];
        for (int i = 0; i < fake.num; ++i)
            if (fake.deleted[i]) {
                lonFakeDVT[i] = -999.0;
                latFakeDVT[i] = -999.0;
            } else {
                lonFakeDVT[i] = fake.DVT[i].point->getCoordinate().getLon()*Rad2Deg;
                latFakeDVT[i] = fake.DVT[i].point->getCoordinate().getLat()*Rad2Deg;
            }
        lonFakeDVTVar->put(lonFakeDVT, fake.num);
        latFakeDVTVar->put(latFakeDVT, fake.num);
    }
    
    NcDim *numIdxDVTDim = file->add_dim("numIdxDVT", this->DT->size()*3);
    NcVar *idxDVTVar = file->add_var("idxDVT", ncInt, numIdxDVTDim);
    int idxDVT[DT->size()*3];
    DelaunayTriangle *DT = this->DT->front();
    int k = -1;
    for (int i = 0; i < this->DT->size(); ++i) {
        for (int j = 0; j < 3; ++j)
            idxDVT[++k] = DT->DVT[j]->getID();
        DT = DT->next;
    }
    idxDVTVar->put(idxDVT, this->DT->size()*3);
    
    file->close();
    delete file;
}
