#ifndef DebugTools_h
#define DebugTools_h

#include "TimeManager.h"
#include "PolygonManager.h"
#include "ApproachDetector.h"
#include <string>
#include <fstream>

using namespace ApproachDetector;

class DebugTools
{
public:
    static void output_angles(Polygon *polygon, const std::string &fileName = "");

    static void output_lengths(Polygon *polygon, const std::string &fileName = "");

    static void output_polygon(const PolygonManager &polygonManager, int timeStep, int ID);

    static bool is_colinear(const Coordinate &x1, const Coordinate &x2,
                            const Coordinate &x3);

#ifdef TTS_ONLINE
    static void assert_consistent_projection(const Projection *projection);
    static void assert_polygon_mass_constant(const PolygonManager &);
    static void assert_polygon_area_constant(const PolygonManager &);
#endif

    static void watch(Vertex *vertex) { watchedVertex = vertex; }
    static void watch(Edge *edge) { watchedEdge = edge; }
    static void watch(Polygon *polygon) { watchedPolygon = polygon; }

    static void dump_watchers();
    static void dump_watched_vertex();
    static void dump_watched_edge();
    static void dump_watched_polygon();

    static Vertex *watchedVertex;
    static Edge *watchedEdge;
    static Polygon *watchedPolygon;
};

#endif
