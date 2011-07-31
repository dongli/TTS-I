/*
 *  Topology.cpp
 *
 *  Created by DONG Li on 11-1-29.
 *  Copyright 2011 LASG/IAP. All rights reserved.
 *
 */

#include "Topology.h"
#include "DelaunayVertex.h"
#include "DelaunayTriangle.h"
#include "ReportMacros.h"
#include <iomanip>

using std::setw;
using std::setprecision;

Topology::Topology()
{
    incidentDT = new List<DelaunayTrianglePointer>(1, 6);
    incidentDT->setName("Incident Delaunay triangles");
    linkDVT = new List<DelaunayVertexPointer>(1, 6);
    linkDVT->setName("Linked Delaunay vertices");
    reinit();
}

Topology::~Topology()
{
    incidentDT->destroy();
    linkDVT->destroy();
}

void Topology::reinit()
{
    isComplete = false;
    incidentDT->recycle();
    linkDVT->recycle();
}

void Topology::mergeIncidentDT(DelaunayTriangle *DT1, DelaunayTriangle *DT2,
                               DelaunayTriangle *DT)
{
    DelaunayTrianglePointer *DTptr = incidentDT->front();
    for (int i = 0; i < incidentDT->size(); ++i) {
        if (DTptr->ptr == DT1 && DTptr->next->ptr == DT2) {
            DTptr->ptr = DT;
            incidentDT->remove(DTptr->next);
            return;
        }
        DTptr = DTptr->next;
    }
    REPORT_ERROR("No incident Delaunay triangle found!")
}

void Topology::splitIncidentDT(DelaunayTriangle *DT,
                               DelaunayTriangle *DT1, DelaunayTriangle *DT2)
{
    DelaunayTrianglePointer *DTptr1, *DTptr2;
    DTptr1 = incidentDT->front();
    for (int i = 0; i < incidentDT->size(); ++i) {
        if (DTptr1->ptr == DT) {
            DTptr1->ptr = DT1;
            incidentDT->insert(DTptr1, &DTptr2);
            DTptr2->ptr = DT2;
            return;
        }
        DTptr1 = DTptr1->next;
    }
    REPORT_ERROR("No incident Delaunay triangle found!")
}

void Topology::deleteLinkDVT(DelaunayVertex *DVT)
{
    DelaunayVertexPointer *DVTptr = linkDVT->front();
    for (int i = 0; i < linkDVT->size(); ++i) {
        if (DVTptr->ptr == DVT) {
            linkDVT->remove(DVTptr);
            return;
        }
        DVTptr = DVTptr->next;
    }
}

void Topology::addLinkDVT(DelaunayVertex *DVT1, DelaunayVertex *DVT2,
                          DelaunayVertex *DVT)
{
    DelaunayVertexPointer *DVTptr1, *DVTptr2;
    DVTptr1 = linkDVT->front();
    for (int i = 0; i < linkDVT->size(); ++i) {
        if (DVTptr1->ptr == DVT1 && DVTptr1->next->ptr == DVT2) {
            linkDVT->insert(DVTptr1, &DVTptr2);
            DVTptr2->ptr = DVT;
            return;
        }
        DVTptr1 = DVTptr1->next;
    }
}

void Topology::extract()
{
    DelaunayTrianglePointer *DTptr = incidentDT->front();
    while (true) {
        DelaunayTriangle *DT = DTptr->ptr;
        int i, j;
        for (i = 0; i < 3; ++i) {
            if (DT->DVT[i] == this->DVT) {
                linkDVT->append();
                j = i != 2 ? i+1 : 0;
                linkDVT->back()->ptr = DT->DVT[j];
                break;
            }
        }
#ifdef DEBUG
        if (i == 3) {
            REPORT_ERROR("No match link Delaunay vertex.")
        }
#endif
        if (incidentDT->front()->ptr == DT->adjDT[j]) {
            // The ring has formed
            incidentDT->ring();
            linkDVT->ring();
            isComplete = true;
            return;
        } else {
            // Shift to the next incident triangle
            incidentDT->append(&DTptr);
            DTptr->ptr = DT->adjDT[j];
        }
    }
}

void Topology::dump()
{
    cout << setw(5) << DVT->getID() << ":" << endl;
    cout << "incidentDT:" << endl;
    DelaunayTrianglePointer *incidentDT;
    incidentDT = DVT->topology.incidentDT->front();
    for (int i = 0; i < DVT->topology.incidentDT->size(); ++i) {
        Point &center = incidentDT->ptr->circumcenter;
        cout << setw(10) << incidentDT->ptr->getID() <<
        setw(20) << setprecision(10) << center.getCoordinate().getLon() <<
        setw(20) << setprecision(10) << center.getCoordinate().getLat() << endl;
        incidentDT = incidentDT->next;
    }
    cout << "linkDVT:" << endl;
    DelaunayVertexPointer *linkDVT;
    linkDVT = DVT->topology.linkDVT->front();
    for (int i = 0; i < DVT->topology.linkDVT->size(); ++i) {
        cout << setw(10) << linkDVT->ptr->getID() << endl;
        linkDVT = linkDVT->next;
    }
}
