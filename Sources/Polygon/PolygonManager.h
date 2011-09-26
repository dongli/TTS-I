#ifndef _PolygonManager_h_
#define _PolygonManager_h_

#include "Vertex.h"
#include "Edge.h"
#include "Polygon.h"
#include <string>

using std::string;

class PolygonManager
{
public:
    PolygonManager();
    virtual ~PolygonManager();

    void init(const string &fileName);

    void output(const string &fileName);

    List<Vertex> vertices;
    List<Edge> edges;
    List<Polygon> polygons;
};

#endif
