#include "DebugTools.h"
#include "Sphere.h"

Vertex *DebugTools::watchedVertex;
Edge *DebugTools::watchedEdge;
Polygon *DebugTools::watchedPolygon;

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

bool DebugTools::is_colinear(const Coordinate &x1, const Coordinate &x2,
                             const Coordinate &x3)
{
    static const double smallDistance = 1.0e-4/Rad2Deg*Sphere::radius;
    Coordinate x2r, x3r;
    Sphere::rotate(x1, x2, x2r);
    Sphere::rotate(x1, x3, x3r);
    if (fabs(x2r.getLon()-x3r.getLon()) > EPS) {
        if (Sphere::calcDistance(x1, x3) < smallDistance ||
            Sphere::calcDistance(x2, x3) < smallDistance)
            return true;
        return false;
    }
    return true;
}

#ifdef TTS_ONLINE
void DebugTools::assert_consistent_projection(const Projection *projection)
{
    TimeLevel timeLevel;
    if (projection->tags.isSet(Approaching)) {
        timeLevel = OldTimeLevel;
    } else {
        timeLevel = NewTimeLevel;
    }
    if (projection->getDistance(timeLevel) == UndefinedDistance) {
        REPORT_WARNING("No projection at all!");
        return;
    }
    Vertex *vertex1 = projection->getEdge()->getEndPoint(FirstPoint);
    Vertex *vertex2 = projection->getEdge()->getEndPoint(SecondPoint);
    const Coordinate &x1 = vertex1->getCoordinate(timeLevel);
    const Coordinate &x2 = vertex2->getCoordinate(timeLevel);
    const Coordinate &x3 = projection->getCoordinate(timeLevel);
    if (!is_colinear(x1, x2, x3)) {
        projection->getVertex()->detectAgent.dump();
        Message message;
        message << "Projection on edge " << projection->getEdge()->getID();
        message << " is not consistent!";
        REPORT_ERROR(message.str());
    }
}

void DebugTools::assert_polygon_mass_constant(const PolygonManager &polygonManager)
{
    double totalMass = 0.0;
    Polygon *polygon = polygonManager.polygons.front();
    for (int i = 0; i < polygonManager.polygons.size(); ++i) {
        if (polygon->tracers.size() != 0)
            totalMass += polygon->tracers[0].getMass();
        polygon = polygon->next;
    }
    double errorMass = (totalMass-385289464684828224.0)/totalMass;
    if (fabs(errorMass) > 1.0e-12) {
        Message message;
        message << "Relative mass error (" << errorMass << ") is too large!";
        REPORT_ERROR(message.str());
    }
}

void DebugTools::assert_polygon_area_constant(const PolygonManager &polygonManager)
{
    static const double trueTotalArea = 4.0*PI*Sphere::radius2;
    double totalArea = 0.0;
    Polygon *polygon = polygonManager.polygons.front();
    for (int i = 0; i < polygonManager.polygons.size(); ++i) {
        if (!polygon->isAreaSet())
            polygon->calcArea();
        totalArea += polygon->getArea();
        polygon = polygon->next;
    }
    double errorArea = fabs(totalArea-trueTotalArea)/trueTotalArea;
    assert(errorArea < 1.0e-6);
}
#endif

void DebugTools::dump_watchers()
{
    dump_watched_vertex();
    dump_watched_edge();
    dump_watched_polygon();
}

void DebugTools::dump_watched_vertex()
{
    if (watchedVertex == NULL || watchedVertex->endTag == Vertex::Null)
        return;
    cout << "Watched vertex ID: " << watchedVertex->getID() << endl;
    cout << "Coordinate:" << endl;
    watchedVertex->getCoordinate().dump();
    watchedVertex->dump();
    std::list<Projection>::const_iterator it;
    for (it = watchedVertex->detectAgent.getProjections().begin();
         it != watchedVertex->detectAgent.getProjections().end(); ++it) {
        const Projection *p = &(*it);
        if (p->tags.isSet(Calculated))
            assert_consistent_projection(p);
    }
    REPORT_DEBUG
}

void DebugTools::dump_watched_edge()
{
    if (watchedEdge == NULL || watchedEdge->endTag == Edge::Null)
        return;
    cout << "Watched edge ID: " << watchedEdge->getID() << endl;
    cout << "  End points ID: ";
    cout << watchedEdge->getEndPoint(FirstPoint)->getID() << "  ";
    cout << watchedEdge->getEndPoint(SecondPoint)->getID() << endl;
    cout << "  Edge pointers: ";
    cout << watchedEdge->getEdgePointer(OrientLeft) << "  ";
    cout << watchedEdge->getEdgePointer(OrientRight) << endl;
    cout << "  Paired vertices: ";
    watchedEdge->detectAgent.dump();
    REPORT_DEBUG
}

void DebugTools::dump_watched_polygon()
{
    if (watchedPolygon == NULL || watchedPolygon->endTag == Polygon::Null)
        return;
    watchedPolygon->dump("watched_polygon");
    REPORT_DEBUG
}
