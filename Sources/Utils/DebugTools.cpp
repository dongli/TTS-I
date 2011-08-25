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

void DebugTools::assert_consistent_projection(Vertex *vertex1,
                                              Vertex *vertex2,
                                              Vertex *vertex3)
{
    TimeLevel timeLevel;
    if (vertex3->detectAgent.isApproaching) {
        timeLevel = OldTimeLevel;
    } else {
        timeLevel = NewTimeLevel;
    }
    const Coordinate &x1 = vertex1->getCoordinate(timeLevel);
    const Coordinate &x2 = vertex2->getCoordinate(timeLevel);
    const Coordinate &x3 = vertex3->detectAgent.projection;
    Coordinate x2r, x3r;
    Sphere::rotate(x1, x2, x2r);
    Sphere::rotate(x1, x3, x3r);
    if (fabs(x2r.getLon()-x3r.getLon()) > EPS) {
        x1.dump();
        x3.dump();
        x2.dump();
        REPORT_ERROR("Projection is not consistent!")
    }
}

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
#ifdef TTS_ONLINE
    cout << "Detector:" << endl;
    if (watcher_vertex->detectAgent.edge != NULL) {
        cout << "  Obstacle edge ID: " << watcher_vertex->detectAgent.edge->getID() << endl;
        cout << "  Approaching?: ";
        if (watcher_vertex->detectAgent.isApproaching)
            cout << "yes" << endl;
        else
            cout << "no" << endl;
        cout << "  End points ID: ";
        cout << watcher_vertex->detectAgent.edge->getEndPoint(FirstPoint)->getID() << " ";
        cout << watcher_vertex->detectAgent.edge->getEndPoint(SecondPoint)->getID() << endl;
    }
#endif
    REPORT_DEBUG
}

void DebugTools::dump_watched_edge()
{
    if (watcher_edge == NULL) {
        return;
    }
    cout << "Watched edge ID: " << watcher_edge->getID() << endl;
    cout << "  End points ID: ";
    cout << watcher_edge->getEndPoint(FirstPoint)->getID() << " ";
    cout << watcher_edge->getEndPoint(SecondPoint)->getID() << endl;
    REPORT_DEBUG
}

void DebugTools::dump_watched_polygon()
{
    if (watcher_polygon == NULL) {
        return;
    }
    watcher_polygon->dump("polygon");
    REPORT_DEBUG
}

Vertex *DebugTools::get_watched_vertex()
{
    return watcher_vertex;
}