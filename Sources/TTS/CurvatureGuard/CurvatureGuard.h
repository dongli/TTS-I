#ifndef CurvatureGuard_h
#define CurvatureGuard_h

class Edge;
class MeshManager;
class FlowManager;
class PolygonManager;

namespace CurvatureGuard
{
    //! \brief The main interface of CurvatureGuard.
    void guard(MeshManager &, const FlowManager &, PolygonManager &);

    //! \brief Split the edge when the line approximation goes poor.
    bool splitEdge(MeshManager &, const FlowManager &, PolygonManager &);
    bool splitEdge(MeshManager &, const FlowManager &, PolygonManager &,
                   Edge *edge, bool isMustSplit = false);

    //! \brief Merge edges when line approximation goes well.
    bool mergeEdge(MeshManager &, const FlowManager &, PolygonManager &);

    //! \brief Split the polygon when vertex-edge approaching events occur.
    bool splitPolygon(MeshManager &, const FlowManager &, PolygonManager &);

    //! \brief Calculate angle thresholds for "splitEdge" and "mergeEdge".
    double angleThreshold(Edge *edge);
    double angleThreshold(Edge *edge1, Edge *edge2);
}

#endif
