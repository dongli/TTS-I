#ifndef _DebugTools_h_
#define _DebugTools_h_

#include "TimeManager.h"
#include "PolygonManager.h"
#include <string>
#include <fstream>

class DebugTools
{
public:
    static void output_angles(Polygon *polygon, const std::string &fileName = "");

    static void output_lengths(Polygon *polygon, const std::string &fileName = "");

    static void output_polygon(const PolygonManager &polygonManager, int timeStep, int ID);

    static void assert_colinear(const Coordinate &x1, const Coordinate &x2,
                                const Coordinate &x3);

    static void watch_vertex(Vertex *);
    static void watch_edge(Edge *);
    static void watch_polygon(Polygon *);

    static void dump_watchers();
    static void dump_watched_vertex();
    static void dump_watched_edge();
    static void dump_watched_polygon();

    static Vertex *get_watched_vertex();

    static Vertex *watcher_vertex;
    static Edge *watcher_edge;
    static Polygon *watcher_polygon;
};

#endif
