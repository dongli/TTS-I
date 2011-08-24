#include "Polygon.h"
#include "Constants.h"
#include "Sphere.h"
#include "TimeManager.h"
#include "PolygonManager.h"
#ifdef TTS_ONLINE
#include "TTS.h"
#endif
#ifdef DEBUG
#include "DebugTools.h"
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
    isAreaSet = false;
}

void Polygon::calcArea()
{
    double excess = 0.0;
    double area;

    EdgePointer *edgePointer = edgePointers.front();
    for (int i = 0; i < edgePointers.size(); ++i) {
#ifdef DEBUG
        assert(edgePointer->getAngle() != UNSET_ANGLE);
#endif
        excess += edgePointer->getAngle();
        edgePointer = edgePointer->next;
    }
    excess -= (edgePointers.size()-2)*PI;
    area = excess*Sphere::radius2;
#ifdef DEBUG
    if (area <= 0.0) {
        cout << "Polygon ID: " << getID() << endl;
        DebugTools::output_angles(this, "angles");
        DebugTools::output_lengths(this, "lengths");
        dump("polygon");
        REPORT_ERROR("Encounter negative area.")
    }
#endif
    if (isAreaSet) {
        this->area.save();
    }
    this->area.setNew(area);
    if (!isAreaSet) {
        this->area.save();
        isAreaSet = true;
    }
#ifdef DEBUG
    static const double maxRatio = 1000.0;
    double ratio = fabs(this->area.getOld()-this->area.getNew())/this->area.getOld();
    if (ratio > maxRatio) {
        cout << "Polygon ID: " << getID() << endl;
        DebugTools::output_angles(this, "angles");
        DebugTools::output_lengths(this, "lengths");
        dump("polygon");
        cout << "Old area: ";
        cout << setw(20) << setprecision(15) << this->area.getOld() << endl;
        cout << "New area: ";
        cout << setw(20) << setprecision(15) << this->area.getNew() << endl;
        char message[100];
        sprintf(message, "Change of polygon area exceeds %f%% (%f%%)!", maxRatio*100.0, ratio*100.0);
        REPORT_WARNING(message);
    }
#endif
}

void Polygon::handleLinePolygon(PolygonManager &polygonManager, Polygon *polygon)
{
    EdgePointer *edgePointer1 = polygon->edgePointers.front();
    EdgePointer *edgePointer2 = polygon->edgePointers.back();
    EdgePointer *edgePointer3;
    polygonManager.polygons.remove(polygon);
    Edge *edge1 = edgePointer1->edge;
    Edge *edge2 = edgePointer2->edge;
#ifdef DEBUG
    assert(((edge1->getEndPoint(FirstPoint) ==
             edge2->getEndPoint(FirstPoint)) &&
            (edge1->getEndPoint(SecondPoint) ==
             edge2->getEndPoint(SecondPoint))) ||
           ((edge1->getEndPoint(FirstPoint) ==
             edge2->getEndPoint(SecondPoint)) &&
            (edge1->getEndPoint(SecondPoint) ==
             edge2->getEndPoint(FirstPoint))));
#endif
    if (edgePointer2->orient == OrientLeft) {
        edgePointer3 = edge2->getEdgePointer(OrientRight);
    } else {
        edgePointer3 = edge2->getEdgePointer(OrientLeft);
    }
    Polygon *polygon2 = edge2->getPolygon(edgePointer3->orient);
    edge1->setPolygon(edgePointer1->orient, polygon2);
    edge1->setEdgePointer(edgePointer1->orient, edgePointer3);
#ifdef TTS_ONLINE
    edge2->detectAgent.handover(edge1);
#endif
    polygonManager.edges.remove(edge2);
#ifdef TTS_ONLINE
    TTS::deleteTask(TTS::UpdateAngle, edgePointer1);
    TTS::deleteTask(TTS::UpdateAngle, edgePointer2);
#endif
}

void Polygon::handlePointPolygon(PolygonManager &polygonManager, Polygon *polygon)
{
#ifdef DEBUG
    assert(polygon->edgePointers.size() == 1);
#endif
    EdgePointer *edgePointer1 = polygon->edgePointers.front();
    EdgePointer *edgePointer2;
    Polygon *polygon2;
    if (edgePointer1->orient == OrientLeft) {
        edgePointer2 = edgePointer1->edge->getEdgePointer(OrientRight);
        polygon2 = edgePointer1->edge->getPolygon(OrientRight);
    } else {
        edgePointer2 = edgePointer1->edge->getEdgePointer(OrientLeft);
        polygon2 = edgePointer1->edge->getPolygon(OrientLeft);
    }
    polygon2->edgePointers.remove(edgePointer2);
#ifdef DEBUG
    assert(edgePointer1->edge->getEndPoint(FirstPoint) ==
           edgePointer1->edge->getEndPoint(SecondPoint));
#endif
    polygonManager.polygons.remove(polygon);
    polygonManager.edges.remove(edgePointer1->edge);
#ifdef TTS_ONLINE
    TTS::deleteTask(TTS::UpdateAngle, edgePointer1);
    TTS::deleteTask(TTS::UpdateAngle, edgePointer2);
#endif
}

void Polygon::dump(const string &fileName) const
{
    std::ostream *output;
    if (fileName != "") {
        output = new std::ofstream(fileName.c_str());
    } else {
        output = &std::cout;
    }
    EdgePointer *edgePointer = edgePointers.front();
    for (int i = 0; i < edgePointers.size(); ++i) {
        const Coordinate &x = edgePointer->getEndPoint(FirstPoint)->getCoordinate();
        *output << setw(30) << setprecision(20) << x.getLon();
        *output << setw(30) << setprecision(20) << x.getLat();
        *output << endl;
        edgePointer = edgePointer->next;
    }
}