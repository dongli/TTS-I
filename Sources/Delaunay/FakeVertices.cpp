/*
 *  FakeVertices.cpp
 *
 *  Created by DONG Li on 11-2-2.
 *  Copyright 2011 LASG/IAP. All rights reserved.
 *
 */

#include "FakeVertices.h"

FakeVertices::FakeVertices()
{
    num = 3;
    deleted = new bool[num];
    DVT = new DelaunayVertex[num];
    for (int i = 0; i < num; ++i) {
        DVT[i].setID(-(i+1));
        DVT[i].point = new Point;
        deleted[i] = false;
    }
}

FakeVertices::~FakeVertices()
{
    delete [] deleted;
    delete [] DVT;
}

void FakeVertices::erase(int i)
{
    delete DVT[i].point;
    deleted[i] = true;
}
