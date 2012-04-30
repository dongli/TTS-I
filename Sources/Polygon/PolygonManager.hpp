#ifndef PolygonManager_h
#define PolygonManager_h

#include "Vertex.hpp"
#include "Edge.hpp"
#include "Polygon.hpp"
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
    void init();
#endif

    void output(const string &fileName);

    List<Vertex> vertices;
    List<Edge> edges;
    List<Polygon> polygons;
};

#endif
