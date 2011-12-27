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

#ifdef TTS_ONLINE
    void init(const string &fileName);
#endif

    void output(const string &fileName);

    List<Vertex> vertices;
    List<Edge> edges;
    List<Polygon> polygons;
};

#endif
