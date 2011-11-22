#include "Polygon.h"
#include "Constants.h"
#include "Sphere.h"
#include "TimeManager.h"
#include "PolygonManager.h"
#ifdef TTS_ONLINE
#include "TTS.h"
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
    isAreaSet = false;
}

void Polygon::calcArea()
{
    double excess = 0.0;
    double area;
    if (edgePointers.size() != 1) {
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
    } else
        area = 0.0;
#ifdef DEBUG
    if (area <= 0.0) {
        cout << "Polygon ID: " << getID() << endl;
#ifdef TTS_ONLINE
        DebugTools::output_angles(this, "angles");
        DebugTools::output_lengths(this, "lengths");
#endif
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
    static const double maxRatio = 5.0;
    double ratio = fabs(this->area.getOld()-this->area.getNew())/this->area.getOld();
    if (ratio > maxRatio) {
        cout << "Polygon ID: " << getID() << endl;
#ifdef TTS_ONLINE
        DebugTools::output_angles(this, "angles");
        DebugTools::output_lengths(this, "lengths");
#endif
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

void Polygon::dump(const char *fileName) const
{
    std::ostream *output = new std::ofstream(fileName);
    dump(output);
    delete output;
}

void Polygon::dump() const
{
    dump(&std::cout);
}

void Polygon::dump(std::ostream *output) const
{
    // vertices
    EdgePointer *edgePointer = edgePointers.front();
    for (int i = 0; i < edgePointers.size(); ++i) {
        const Coordinate &x = edgePointer->getEndPoint(FirstPoint)->getCoordinate();
        *output << setw(5) << 0;
        *output << setw(30) << setprecision(20) << x.getLon();
        *output << setw(30) << setprecision(20) << x.getLat();
        *output << endl;
        edgePointer = edgePointer->next;
    }
    // test points
    edgePointer = edgePointers.front();
    for (int i = 0; i < edgePointers.size(); ++i) {
        const Coordinate &x = edgePointer->edge->getTestPoint()->getCoordinate();
        *output << setw(5) << 1;
        *output << setw(30) << setprecision(20) << x.getLon();
        *output << setw(30) << setprecision(20) << x.getLat();
        *output << endl;
        edgePointer = edgePointer->next;
    }
}