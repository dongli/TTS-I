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
    }
    reinit();
}

FakeVertices::~FakeVertices()
{
    delete [] deleted;
    for (int i = 0; i < num; ++i)
        delete DVT[i].point;
    delete [] DVT;
}

void FakeVertices::reinit()
{
    for (int i = 0; i < num; ++i) {
        deleted[i] = false;
        DVT[i].reinit();
    }
}

void FakeVertices::erase(int i)
{
    deleted[i] = true;
}
