#include "DebugTools.h"
#include "Sphere.h"

Vertex *DebugTools::watcher_vertex;
Edge *DebugTools::watcher_edge;
Polygon *DebugTools::watcher_polygon;

void DebugTools::output_angles(Polygon *polygon, const std::string &fileName)
{
    std::ostream *output;
    if (fileName != "") {
        output = new std::ofstream(fileName.c_str());
    } else {
        output = &std::cout;
    }
    if (fileName == "")
        *output << "Angles:" << endl;
    EdgePointer *edgePointer = polygon->edgePointers.front();
    for (int i = 0; i < polygon->edgePointers.size(); ++i) {
        *output << edgePointer->getAngle(OldTimeLevel)*Rad2Deg << "  " <<
        edgePointer->getAngle(NewTimeLevel)*Rad2Deg << "  " <<
        (edgePointer->getAngle(NewTimeLevel)-
         edgePointer->getAngle(OldTimeLevel))*Rad2Deg << endl;
        edgePointer = edgePointer->next;
    }
}

void DebugTools::output_lengths(Polygon *polygon, const std::string &fileName)
{
    std::ostream *output;
    if (fileName != "") {
        output = new std::ofstream(fileName.c_str());
    } else {
        output = &std::cout;
    }
    if (fileName == "")
        *output << "Lengths:" << endl;
    EdgePointer *edgePointer = polygon->edgePointers.front();
    for (int i = 0; i < polygon->edgePointers.size(); ++i) {
        *output << edgePointer->edge->getLength() << endl;
        edgePointer = edgePointer->next;
    }
}

void DebugTools::output_polygon(const PolygonManager &polygonManager, int timeStep, int ID)
{
    if (TimeManager::getSteps() == timeStep) {
        Polygon *polygon = polygonManager.polygons.front();
        for (int i = 0; i < polygonManager.polygons.size(); ++i) {
            if (polygon->getID() == ID) {
                char fileName[30];
                sprintf(fileName, "polygon_%d", ID);
                polygon->dump(fileName);
                break;
            }
            polygon = polygon->next;
        }
        REPORT_DEBUG
    }
}

void DebugTools::assert_colinear(const Coordinate &x1, const Coordinate &x2,
                                 const Coordinate &x3)
{
    Coordinate x2r, x3r;
    Sphere::rotate(x1, x2, x2r);
    Sphere::rotate(x1, x3, x3r);
    assert(fabs(x2r.getLon()-x3r.getLon()) < EPS);
}

#ifdef TTS_ONLINE
void DebugTools::assert_consistent_projection(Projection *projection)
{
    TimeLevel timeLevel;
    if (projection->isApproaching()) {
        timeLevel = OldTimeLevel;
    } else {
        timeLevel = NewTimeLevel;
    }
    Vertex *vertex1 = projection->getEdge()->getEndPoint(FirstPoint);
    Vertex *vertex2 = projection->getEdge()->getEndPoint(SecondPoint);
    const Coordinate &x1 = vertex1->getCoordinate(timeLevel);
    const Coordinate &x2 = vertex2->getCoordinate(timeLevel);
    const Coordinate &x3 = projection->getCoordinate(timeLevel);
    Coordinate x2r, x3r;
    Sphere::rotate(x1, x2, x2r);
    Sphere::rotate(x1, x3, x3r);
    if (fabs(x2r.getLon()-x3r.getLon()) > EPS) {
        projection->getVertex()->detectAgent.dump();
        REPORT_ERROR("Projection is not consistent!")
    }
}
#endif

void DebugTools::watch_vertex(Vertex *vertex)
{
    watcher_vertex = vertex;
}

void DebugTools::watch_polygon(Polygon *polygon)
{
    watcher_polygon = polygon;
}

void DebugTools::watch_edge(Edge *edge)
{
    watcher_edge = edge;
}

void DebugTools::dump_watchers()
{
    dump_watched_vertex();
    dump_watched_edge();
    dump_watched_polygon();
}

void DebugTools::dump_watched_vertex()
{
    if (watcher_vertex == NULL) {
        //REPORT_ERROR("Watched vertex is NULL!")
        return;
    }
    cout << "Watched vertex ID: " << watcher_vertex->getID() << endl;
    cout << "Coordinate:" << endl;
    watcher_vertex->getCoordinate().dump();
    watcher_vertex->dump();
    REPORT_DEBUG
}

void DebugTools::dump_watched_edge()
{
    if (watcher_edge == NULL) {
        return;
    }
    cout << "Watched edge ID: " << watcher_edge->getID() << endl;
    cout << "  End points ID: ";
    cout << watcher_edge->getEndPoint(FirstPoint)->getID() << "  ";
    cout << watcher_edge->getEndPoint(SecondPoint)->getID() << endl;
    cout << "  Edge pointers: ";
    cout << watcher_edge->getEdgePointer(OrientLeft) << "  ";
    cout << watcher_edge->getEdgePointer(OrientRight) << endl;
    REPORT_DEBUG
}

void DebugTools::dump_watched_polygon()
{
    if (watcher_polygon == NULL) {
        return;
    }
    watcher_polygon->dump("watched_polygon");
    REPORT_DEBUG
}

Vertex *DebugTools::get_watched_vertex()
{
    return watcher_vertex;
}