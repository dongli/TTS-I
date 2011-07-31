#ifndef _process_cocircularDT_h_
#define _process_cocircularDT_h_

#include "DelaunayDriver.h"
#include "CocircularDTGroups.h"
#include "Constants.h"
#include <cmath>

inline void process_cocircularDT(DelaunayDriver &driver)
{
    CocircularDTGroups groups;
    DelaunayVertex *DVT;
    DelaunayTrianglePointer *incidentDT;
    DelaunayTrianglePointer *nextIncidentDT;

    DVT = driver.DVT->front();
    for (int i = 0; i < driver.DVT->size(); ++i) {
        // Merge the circumcenters that are too close
        incidentDT = DVT->topology.incidentDT->front();
        bool flag = false;
        do {
            nextIncidentDT = incidentDT->next;
            Point &center1 = incidentDT->ptr->circumcenter;
            Point &center2 = nextIncidentDT->ptr->circumcenter;
            double dlon = center1.getCoordinate().getLon()-
                          center2.getCoordinate().getLon();
            double dlat = center1.getCoordinate().getLat()-
                          center2.getCoordinate().getLat();
            if (fabs(dlat) < 1.0e-6) {
                if (PI05-fabs(center1.getCoordinate().getLat()) < 1.0e-6 ||
                    (fabs(dlon) < 1.0e-6 || PI2-fabs(dlon) < 1.0e-6)) {
                    DelaunayTriangle *leadingDT;
                    DelaunayTriangle *removedDT;
                    // Record the two cocircular DT and choose the 
                    // leading DT to represent them (There may be already
                    // other DT that are cocircular with the two)
                    groups.record(incidentDT->ptr, nextIncidentDT->ptr,
                                  &leadingDT, &removedDT);
                    // Find the pair DVT which is also facing the
                    // cocircular DT
                    DelaunayVertex *pairDVT;
                    // There are also two victim DVT that are affected
                    // by the merge operation
                    DelaunayVertex *victimDVT1, *victimDVT2;
                    int j1, j2, j1m1, j1p1, j2m1, j2p1;
                    for (j1 = 0; j1 < 3; ++j1)
                        if (leadingDT->DVT[j1] == DVT) break;
#ifdef DEBUG
                    if (j1 == 3) {
                        REPORT_ERROR("No matched DVT in leadingDT.")
                    }
#endif
                    for (j2 = 0; j2 < 3; ++j2)
                        if (removedDT->DVT[j2] == DVT) break;
#ifdef DEBUG
                    if (j2 == 3) {
                        REPORT_ERROR("No matched DVT in removedDT.")
                    }
#endif
                    j1m1 = j1 != 0 ? j1-1 : 2;
                    j1p1 = j1 != 2 ? j1+1 : 0;
                    j2m1 = j2 != 0 ? j2-1 : 2;
                    j2p1 = j2 != 2 ? j2+1 : 0;
                    if (leadingDT == incidentDT->ptr) {
                        //pairDVT = leadingDT->DVT[j1m1];
                        pairDVT = removedDT->DVT[j2p1]; // This is buggy!
                        victimDVT1 = leadingDT->DVT[j1p1];
                        victimDVT2 = removedDT->DVT[j2m1];
                        DVT->topology.incidentDT->remove(nextIncidentDT);
                    } else if (leadingDT == nextIncidentDT->ptr) {
                        //pairDVT = leadingDT->DVT[j1p1];
                        pairDVT = removedDT->DVT[j2m1]; // This is buggy!
                        victimDVT1 = leadingDT->DVT[j1m1];
                        victimDVT2 = removedDT->DVT[j2p1];
                        DVT->topology.incidentDT->remove(incidentDT);
                        incidentDT = nextIncidentDT;
                    }
#ifdef DEBUG
                    int unexpectedJ = pairDVT->topology.incidentDT->size();
#endif
                    DelaunayTrianglePointer *tempDT;
                    tempDT = pairDVT->topology.incidentDT->front();
                    int j;
                    for (j = 0; j < pairDVT->topology.incidentDT->size(); ++j) {
                        if (tempDT->ptr == removedDT) {
                            pairDVT->topology.incidentDT->remove(tempDT);
                            break;
                        }
                        tempDT = tempDT->next;
                    }
#ifdef DEBUG
                    if (j == unexpectedJ) {
                        REPORT_ERROR("No matched removedDT in pairDVT's incidentDT.")
                    }
#endif
                    // This is buggy! How to delete linkDVT and keep the correct
                    // order of the rest ones?
                    DVT->topology.deleteLinkDVT(pairDVT);
                    pairDVT->topology.deleteLinkDVT(DVT);
#ifdef DEBUG
                    unexpectedJ = victimDVT1->topology.incidentDT->size();
#endif
                    // This is buggy! Synchronize the victimDVT
                    tempDT = victimDVT1->topology.incidentDT->front();
                    for (j = 0; j < victimDVT1->topology.incidentDT->size(); ++j) {
                        if (tempDT->ptr == removedDT) {
                            tempDT->ptr = leadingDT;
                            break;
                        } else if (tempDT->ptr == leadingDT) {
                            break;
                        }
                        tempDT = tempDT->next;
                    }
#ifdef DEBUG
                    if (j == unexpectedJ) {
                        REPORT_ERROR("No matched removeDT in victimDVT's incidentDT.")
                    }
#endif
#ifdef DEBUG
                    unexpectedJ = victimDVT2->topology.incidentDT->size();
#endif
                    tempDT = victimDVT2->topology.incidentDT->front();
                    for (int j = 0; j < victimDVT2->topology.incidentDT->size(); ++j) {
                        if (tempDT->ptr == removedDT) {
                            tempDT->ptr = leadingDT;
                            break;
                        } else if (tempDT->ptr == leadingDT) {
                            break;
                        }
                        tempDT = tempDT->next;
                    }
#ifdef DEBUG
                    if (j == unexpectedJ) {
                        REPORT_ERROR("No matched removeDT in victimDVT's incidentDT.")
                    }
#endif
                } else {
                    incidentDT = incidentDT->next;
                    flag = true;
                }
            } else {
                incidentDT = incidentDT->next;
                flag = true;
            }
            if (flag && incidentDT == DVT->topology.incidentDT->front()) break;
        } while (true);
        DVT = DVT->next;
    }
    groups.reduce(driver.DT);

    // Make sure the order of the linkDVT is matched with
    // the order of the incidentDT
    DVT = driver.DVT->front();
    for (int i = 0; i < driver.DVT->size(); ++i) {
        DelaunayVertexPointer *linkDVT1;
        DelaunayTrianglePointer *incidentDT1;
        linkDVT1 = DVT->topology.linkDVT->front();
        incidentDT1 = DVT->topology.incidentDT->front();
        for (int j = 0; j < DVT->topology.linkDVT->size(); ++j) {
            DelaunayVertexPointer *linkDVT2;
            DelaunayTrianglePointer *incidentDT2;
            linkDVT2 = linkDVT1->ptr->topology.linkDVT->front();
            incidentDT2 = linkDVT1->ptr->topology.incidentDT->front();
            for (int k = 0; k < linkDVT1->ptr->topology.linkDVT->size(); ++k) {
                if (linkDVT2->ptr == DVT) break;
                linkDVT2 = linkDVT2->next;
                incidentDT2 = incidentDT2->next;
            }
            int ID1 = incidentDT1->prev->ptr->getID();
            int ID2 = incidentDT1->ptr->getID();
            int ID3 = incidentDT2->ptr->getID();
            if (ID1 != ID3) {
                if (ID2 != ID3) {
                    REPORT_ERROR("Two DVT do not share an edge!")
                }
                if (incidentDT2->next->ptr->getID() == ID1) {
                    linkDVT1->ptr->topology.incidentDT->shift(1);
                }
            }
            linkDVT1 = linkDVT1->next;
            incidentDT1 = incidentDT1->next;
        }
        DVT = DVT->next;
    }
}

#endif
