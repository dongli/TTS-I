/*
 *  CocircularDTGroups.cpp
 *
 *  Created by DONG Li on 11-2-22.
 *  Copyright 2011 LASG/IAP. All rights reserved.
 *
 */

#include "CocircularDTGroups.h"
#include <algorithm>

CocircularDTGroups::CocircularDTGroups()
{
}

CocircularDTGroups::~CocircularDTGroups()
{
}

void CocircularDTGroups::record(DelaunayTriangle *DT1, DelaunayTriangle *DT2,
                                DelaunayTriangle **leadingDT,
                                DelaunayTriangle **removedDT)
{
    list<list<DelaunayTriangle *> >::iterator it = cocircularDT.begin();
    for (; it != cocircularDT.end(); ++it) {
        if (find((*it).begin(), (*it).end(), DT1) != (*it).end()) {
            *leadingDT = *((*it).begin());
            if (DT1 == *leadingDT) {
                *removedDT = DT2;
            } else if (DT2 == *leadingDT) {
                *removedDT = DT1;
            } else {
                REPORT_ERROR("How to handle this branch?")
            }
            
            //REPORT_DEBUG // Keep this block for a while
            //cout << (*((*it).begin()))->getID() << endl;
            
            // This is not debug
            (*it).push_back(DT2);
            
            //cout << DT1->getID() << endl;
            //cout << DT2->getID() << endl;
            //cout << (*((*it).begin()))->getID() << endl;
            
            return;
        }
        if (find((*it).begin(), (*it).end(), DT2) != (*it).end()) {
            *leadingDT = *((*it).begin());
            if (DT1 == *leadingDT) {
                *removedDT = DT2;
            } else if (DT2 == *leadingDT) {
                *removedDT = DT1;
            } else {
                REPORT_ERROR("How to handle this branch?")
            }
            
            //REPORT_DEBUG // Keep this block for a while
            //cout << (*((*it).begin()))->getID() << endl;
            
            // This is not debug
            (*it).push_back(DT1);
            
            //cout << DT1->getID() << endl;
            //cout << DT2->getID() << endl;
            //cout << (*((*it).begin()))->getID() << endl;
            
            return;
        }
    }
    *leadingDT = DT1;
    *removedDT = DT2;
    list<DelaunayTriangle *> newSet;
    newSet.push_back(DT1);
    newSet.push_back(DT2);
    cocircularDT.push_back(newSet);
    
    /*REPORT_DEBUG  // Keep this block for a while
    cout << DT1->getID() << endl;
    cout << DT2->getID() << endl;
    cout << (*(cocircularDT.back().begin()))->getID() << endl;
    return;*/
}

int CocircularDTGroups::reduce(List<DelaunayTriangle> *DT)
{
    int numReducedDT = 0;
    list<list<DelaunayTriangle *> >::iterator it1 = cocircularDT.begin();
    for (; it1 != cocircularDT.end(); ++it1) {
        list<DelaunayTriangle *>::iterator it2 = ++(*it1).begin();
        for (; it2 != (*it1).end(); ++it2) {
            DT->remove((*it2));
            ++numReducedDT;
        }
    }
    DT->reindex();
    return numReducedDT;
}
