/*
 *  FakeVertices.h
 *
 *  Created by DONG Li on 11-2-2.
 *  Copyright 2011 LASG/IAP. All rights reserved.
 *
 */

#ifndef _FakeVertices_h_
#define _FakeVertices_h_

#include "DelaunayVertex.h"

class FakeVertices
{
public:
    FakeVertices();
    virtual ~FakeVertices();

    void reinit();
    void erase(int);
    
    int num;
    bool *deleted;
    DelaunayVertex *DVT;
};

#endif
