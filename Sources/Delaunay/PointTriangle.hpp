/*
 *  PointTriangle.h
 *
 *  Created by DONG Li on 11-1-27.
 *  Copyright 2011 LASG/IAP. All rights reserved.
 *
 */

#ifndef PointTriangle_h
#define PointTriangle_h

#include "List.hpp"

class DelaunayVertex;
class DelaunayVertexPointer;
class DelaunayTriangle;
class DelaunayTrianglePointer;

/*
 * Class:
 *   PointTriangle
 * Purpose:
 *   Provides relevant operations on recording and removing uninserted
 *   points into triangles.
 */
class PointTriangle
{
public:
    static void recordPoint(DelaunayVertex *, DelaunayTriangle *);
    static void recordPoint(DelaunayVertex *, DelaunayTriangle *,
                            DelaunayTriangle *, int);
    static void removePoint(DelaunayVertex *);
};

/*
 * Class:
 *   PIT - Point In Triangle
 * Purpose:
 *   Record the Delaunay triangles that include the uninserted point.
 */
class PIT
{
public:
    PIT();
    virtual ~PIT();
    
    void reinit();
    
    DelaunayVertex *DVT;
    DelaunayTriangle *DT[2];
    DelaunayVertexPointer *stub[2];                                  
    int edgeIdx;
};

/*
 * Class:
 *   TIP - Triangle Includes Points
 * Purpose:
 *   Record the uninserted points to the Delaunay triangle that includes them.
 */
class TIP
{
public:
    TIP();
    virtual ~TIP();
    
    void reinit();
    bool handover(DelaunayVertex *);
    
    DelaunayTriangle *DT;
    List<DelaunayVertexPointer> *points;
};
    
#endif
