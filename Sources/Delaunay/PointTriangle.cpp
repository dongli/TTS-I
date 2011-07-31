/*
 *  PointTriangle.cpp
 *
 *  Created by DONG Li on 11-1-28.
 *  Copyright 2011 LASG/IAP. All rights reserved.
 *
 */

#include "PointTriangle.h"
#include "DelaunayVertex.h"
#include "DelaunayTriangle.h"
#include "Sphere.h"

// *****************************************************************************

PIT::PIT()
{
    DVT = NULL;
    reinit();
}

PIT::~PIT()
{
}

void PIT::reinit()
{
    DT[0] = NULL;
    DT[1] = NULL;
    stub[0] = NULL;
    stub[1] = NULL;
    edgeIdx = -1;
}

// *****************************************************************************

TIP::TIP()
{
    points = new List<DelaunayVertexPointer>(10, 10);
    points->setName("Included points for inserting");
    reinit();
}

TIP::~TIP()
{
    points->destroy();
    delete points;
}

void TIP::reinit()
{
    points->recycle();
}

bool TIP::handover(DelaunayVertex *point)
{
    bool success;
    int ret, onPlane[2];
    
    if (DT->subDT[0] != NULL) {
        // The triangle has been subdivided, go to next level.
        for (int i = 0; i < 3; ++i) {
            if (DT->subDT[i] != NULL)
                success = DT->subDT[i]->tip.handover(point);
            if (success) return true;
        }
        return false;
    } else {
        // The triangle at the bottom level without being subdivided.
        onPlane[0] = -1; onPlane[1] = -1;
        // Check edge 3->1
        ret = Sphere::orient(DT->DVT[2]->point, DT->DVT[0]->point, point->point);
        if (ret == OrientRight) {
            return false;
        } else if (ret == OrientOn) {
            onPlane[0] = 1;
        }
        // Check edge 2->3
        ret = Sphere::orient(DT->DVT[1]->point, DT->DVT[2]->point, point->point);
        if (ret == OrientRight) {
            return false;
        } else if (ret == OrientOn) {
            onPlane[1] = 0;
        }
        // Summary.
        // Note: Delete point from the triangles to avoid duplicate update.
        PointTriangle::removePoint(point);
        if (onPlane[0] == -1 && onPlane[1] == -1) {
            PointTriangle::recordPoint(point, DT);
        } else if (onPlane[0] == 1 && onPlane[1] == -1) {
            PointTriangle::recordPoint(point, DT, DT->adjDT[1], onPlane[0]);
        } else if (onPlane[0] == -1 && onPlane[1] == 0) {
            PointTriangle::recordPoint(point, DT, DT->adjDT[0], onPlane[1]);
        } else {
            cout << "[Error]: DelaunayTriangle::handover: " <<
                    "Point coincides with vertex 3 of triangle " <<
                    DT->getID() << ".\n";
            exit(1);
        }
        return true;
    }
    return false;
}

// *****************************************************************************

void PointTriangle::recordPoint(DelaunayVertex *DVT, DelaunayTriangle *DT)
{
    DVT->pit.DT[0] = DT;
    DVT->pit.DT[1] = NULL;
    DT->tip.points->append();
    DT->tip.points->back()->ptr = DVT;
    DVT->pit.stub[0] = DT->tip.points->back();
    DVT->pit.stub[1] = NULL;
}

void PointTriangle::recordPoint(DelaunayVertex *DVT, DelaunayTriangle *DT1,
                                DelaunayTriangle *DT2, int edgeIdx)
{
    DVT->pit.DT[0] = DT1;
    DVT->pit.DT[1] = DT2;
    DT1->tip.points->append();
    DT1->tip.points->back()->ptr = DVT;
    DT2->tip.points->append();
    DT2->tip.points->back()->ptr = DVT;
    DVT->pit.stub[0] = DT1->tip.points->back();
    DVT->pit.stub[1] = DT2->tip.points->back();
    DVT->pit.edgeIdx = edgeIdx;
}

void PointTriangle::removePoint(DelaunayVertex *DVT)
{
    if (DVT->pit.DT[0] != NULL) {
        DVT->pit.DT[0]->tip.points->erase(DVT->pit.stub[0]);
        DVT->pit.stub[0] = NULL;
        DVT->pit.DT[0] = NULL;
    }
    if (DVT->pit.DT[1] != NULL) {
        DVT->pit.DT[1]->tip.points->erase(DVT->pit.stub[1]);
        DVT->pit.stub[1] = NULL;
        DVT->pit.DT[1] = NULL;
    }
}
