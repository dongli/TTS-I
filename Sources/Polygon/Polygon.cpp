#include "Polygon.h"
#include "Constants.h"
#include "Sphere.h"
#include "TimeManager.h"
#include "PolygonManager.h"
#ifdef TTS_ONLINE
#include "TTS.h"
#include "CommonTasks.h"
#ifdef DEBUG
#include "DebugTools.h"
#endif
#endif
#include <fstream>

Polygon::Polygon()
{
    area.init();
    reinit();
}

Polygon::~Polygon()
{
    edgePointers.destroy();
}

void Polygon::reinit()
{
    edgePointers.recycle();
#ifdef TTS_ONLINE
    for (int i = 0; i < tracers.size(); ++i)
        tracers[i].reinit();
#endif
    areaSet = false;
}

void Polygon::destroy()
{
    EdgePointer *edgePointer = edgePointers.front();
    for (int i = 0; i < edgePointers.size(); ++i) {
#ifdef TTS_ONLINE
        CommonTasks::recordTask(CommonTasks::RemoveObject, edgePointer->edge);
        CommonTasks::recordTask(CommonTasks::RemoveObject, edgePointer->getEndPoint(FirstPoint));
#endif
        edgePointer = edgePointer->next;
    }
}

void Polygon::removeEdge(EdgePointer *edgePointer,
                         PolygonManager &polygonManager)
{
#ifdef DEBUG
    assert(edgePointer->getPolygon(OrientLeft) == this);
#endif
    Vertex *vertex1 = edgePointer->getEndPoint(FirstPoint);
    Vertex *vertex2 = edgePointer->getEndPoint(SecondPoint);
    Edge *edge = edgePointer->edge;
    Polygon *neighborPolygon = edgePointer->getPolygon(OrientRight);
    // keep first end point
    neighborPolygon->edgePointers.remove(edgePointer->getNeighborEdgePointer());
    this->edgePointers.remove(edgePointer);
    polygonManager.edges.remove(edge);
    vertex2->handoverEdges(vertex1, polygonManager);
    polygonManager.vertices.remove(vertex2);
}

void Polygon::calcArea()
{
    double excess = 0.0;
    double area;
    if (edgePointers.size() != 1) {
        EdgePointer *edgePointer = edgePointers.front();
        for (int i = 0; i < edgePointers.size(); ++i) {
#ifdef DEBUG
            if (edgePointer->getAngle() == UNSET_ANGLE) {
                Message message;
                message << "Polygon " << getID() << " has unset angle between ";
                message << "edge " << edgePointer->prev->edge->getID();
                message << " and " << edgePointer->edge->getID() << "!";
                REPORT_ERROR(message.str());
            }
#endif
            excess += edgePointer->getAngle();
            edgePointer = edgePointer->next;
        }
        excess -= (edgePointers.size()-2)*PI;
        area = excess*Sphere::radius2;
#ifdef DEBUG
        if (fabs(excess) > 1.0 || excess <= 0.0)
            REPORT_DEBUG;
#endif
    } else
        area = 0.0;
    if (areaSet)
        this->area.save();
    this->area.setNew(area);
    if (!areaSet) {
        this->area.save();
        areaSet = true;
    }
}

#ifdef TTS_ONLINE
void Polygon::updateTracer(int tracerId)
{
    tracers[tracerId].setDensity(tracers[tracerId].getMass()/area.getNew());
}

void Polygon::updateTracers()
{
    for (int i = 0; i < tracers.size(); ++i) {
        tracers[i].setDensity(tracers[i].getMass()/area.getNew());
    }
}

void Polygon::handoverTracers()
{
    for (int i = 0; i < tracers.size(); ++i) {
        int count = 0;
        Polygon *polygons[edgePointers.size()];
        EdgePointer *edgePointer = edgePointers.front();
        for (int j = 0; j < edgePointers.size(); ++j) {
            // exclude NULL and removed polygons
            if (edgePointer->getPolygon(OrientRight) != NULL &&
                edgePointer->getPolygon(OrientRight)->endTag != Null) {
                polygons[count++] = edgePointer->getPolygon(OrientRight);
            }
            edgePointer = edgePointer->next;
        }
#ifdef DEBUG
        assert(count != 0);
#endif
        double mass = tracers[i].getMass()/count;
        for (int j = 0; j < count; ++j) {
            polygons[j]->tracers[i].addMass(mass);
        }
        tracers[i].setMass(0.0);
    }
}

void Polygon::handoverTracers(Polygon *polygon, double percent)
{
    if (polygon->tracers.size() == 0)
        // polygon is newly created, no tracer has been added
        polygon->tracers.resize(tracers.size());
    for (int i = 0; i < tracers.size(); ++i) {
        double mass1 = tracers[i].getMass()*percent;
        double mass2 = tracers[i].getMass()-mass1;
        polygon->tracers[i].addMass(mass1);
        tracers[i].setMass(mass2);
    }
}
#endif

void Polygon::dump(const char *fileName, TimeLevel timeLevel) const
{
    std::ostream *output = new std::ofstream(fileName);
    dump(output, timeLevel);
    delete output;
}

void Polygon::dump() const
{
    dump(&std::cout);
}

void Polygon::dump(std::ostream *output, TimeLevel timeLevel) const
{
    // vertices
    EdgePointer *edgePointer = edgePointers.front();
    for (int i = 0; i < edgePointers.size(); ++i) {
        const Coordinate &x = edgePointer->getEndPoint(FirstPoint)
        ->getCoordinate(timeLevel);
        *output << setw(5) << 0;
        *output << setw(30) << setprecision(20) << x.getLon();
        *output << setw(30) << setprecision(20) << x.getLat();
        *output << endl;
        edgePointer = edgePointer->next;
    }
#ifdef TTS_ONLINE
    // test points
    edgePointer = edgePointers.front();
    for (int i = 0; i < edgePointers.size(); ++i) {
        const Coordinate &x = edgePointer->edge->getTestPoint()
        ->getCoordinate(timeLevel);
        *output << setw(5) << 1;
        *output << setw(30) << setprecision(20) << x.getLon();
        *output << setw(30) << setprecision(20) << x.getLat();
        *output << endl;
        edgePointer = edgePointer->next;
    }
#endif
}