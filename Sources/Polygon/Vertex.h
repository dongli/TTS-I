#ifndef _Vertex_h_
#define _Vertex_h_

#include "Point.h"
#include "List.h"
#ifdef TTS_ONLINE
class MeshManager;
class FlowManager;
class PolygonManager;
#include "ApproachDetector.h"
#endif

class Edge;
class EdgePointer;

class Vertex : public Point, public ListElement<Vertex>
{
public:
	Vertex();
	virtual ~Vertex();

    void reinit();
    void clean();

    void linkEdge(Edge *);
    void dislinkEdge(Edge *);
#ifdef TTS_ONLINE
    void handoverEdges(Vertex *, MeshManager &, const FlowManager &,
                       PolygonManager &);
#endif

    bool isJoint() const { return linkedEdges.size() > 2; }

    Vertex &operator=(const Vertex &);

    void dump(int indentLevel = 0) const;

#ifdef TTS_ONLINE
    ApproachDetector::VertexAgent detectAgent;
#endif

	List<EdgePointer> linkedEdges;
};

class VertexPointer : public ListElement<VertexPointer>
{
public:
    VertexPointer();
    virtual ~VertexPointer();

    void reinit();

    Vertex *vertex;
};

#endif
