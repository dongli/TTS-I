/*
 *  CocircularDTGroups.h
 *
 *  Created by DONG Li on 11-2-22.
 *  Copyright 2011 LASG/IAP. All rights reserved.
 *
 */

#ifndef _CocircularDTGroups_h_
#define _CocircularDTGroups_h_

#include "DelaunayTriangle.h"
#include <list>
#include <set>

using std::list;
using std::set;

class CocircularDTGroups
{
public:
    CocircularDTGroups();
    virtual ~CocircularDTGroups();

    void record(DelaunayTriangle *, DelaunayTriangle *,
                DelaunayTriangle **, DelaunayTriangle **);
    int reduce(List<DelaunayTriangle> *);
    
    // Note: the inner list only contains two element constantly!
    list<list<DelaunayTriangle *> > cocircularDT;
};

#endif
