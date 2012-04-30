/*
 *  Topology.h
 *
 *  Created by DONG Li on 11-1-29.
 *  Copyright 2011 LASG/IAP. All rights reserved.
 *
 */

#ifndef Topology_h
#define Topology_h

#include "List.hpp"

class DelaunayVertex;
class DelaunayVertexPointer;
class DelaunayTriangle;
class DelaunayTrianglePointer;

/*
 * Class:
 *   Topology
 * Purpose:
 *   Describe the topology of the Delaunay vertex.
 */
class Topology
{
public:
    Topology();
    virtual ~Topology();
    
    void reinit();
    
    void mergeIncidentDT(DelaunayTriangle *, DelaunayTriangle *, DelaunayTriangle *);
    void splitIncidentDT(DelaunayTriangle *, DelaunayTriangle *, DelaunayTriangle *);
    void deleteLinkDVT(DelaunayVertex *);
    void addLinkDVT(DelaunayVertex *, DelaunayVertex *, DelaunayVertex *);

    void extract();
    
    void dump();

    DelaunayVertex *DVT;
    bool isComplete; 
    List<DelaunayTrianglePointer> incidentDT;                    
    List<DelaunayVertexPointer> linkDVT;
};

#endif
