#ifndef _Vertex_h_
#define _Vertex_h_

#include "Point.h"
#include "List.h"

class Edge;
class EdgePointer;

class Vertex : public Point, public ListElement<Vertex>
{
public:
	Vertex();
	virtual ~Vertex();

    void reinit();

    void linkEdge(Edge *);
    void dislinkEdge(Edge *);

    bool isJoint() const { return edgePointers.size() > 2; }

    Vertex &operator=(const Vertex &);

    void dump(int indentLevel = 0) const;

private:
	List<EdgePointer> edgePointers;
};

#endif
