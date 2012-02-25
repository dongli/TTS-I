#ifndef _PolygonManager_h_
#define _PolygonManager_h_

#include "Vertex.h"
#include "Edge.h"
#include "Polygon.h"
#include <string>

using std::string;

class DelaunayDriver;

class PolygonManager
{
public:
    PolygonManager();
    virtual ~PolygonManager();
    
    void reinit();
    void init(const DelaunayDriver &);
#ifdef TTS_ONLINE
    void init(const string &fileName);
#endif

    void output(const string &fileName);

    List<Vertex> vertices;
    List<Edge> edges;
    List<Polygon> polygons;
};

#endif
