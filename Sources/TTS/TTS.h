#ifndef _TTS_h_
#define _TTS_h_

#include "MeshManager.h"
#include "FlowManager.h"
#include "ParcelManager.h"

class TTS
{
public:
    TTS();
    virtual ~TTS();


    void advect(MeshManager &, const FlowManager &, ParcelManager &);

private:
    void track(MeshManager &, const FlowManager &, Point *, int ID = -1) const;

    double angleThreshold(Edge *edge);
    double angleThreshold(Edge *edge1, Edge *edge2);

    void changeEdgeEndPoint(Edge *, PointOrder, Vertex *, MeshManager &,
                            const FlowManager &) const;

    bool splitEdge(MeshManager &, const FlowManager &,
                   PolygonManager &, Edge *);
    bool mergeEdge(MeshManager &, const FlowManager &,
                   PolygonManager &, Polygon *);
    bool handleSmallEdge(MeshManager &, const FlowManager &,
                         PolygonManager &, Edge *);
    bool handleWrongAngle(MeshManager &, const FlowManager &,
                          PolygonManager &, Polygon *);

    void guardCurvature(MeshManager &, const FlowManager &,
                        PolygonManager &);

    double A0, A1, dA;
    double L0, L1, dL;
};

#endif
