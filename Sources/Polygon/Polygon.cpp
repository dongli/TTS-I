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
        excess = 0.0;
        edgePointer = edgePointers.front();
        for (int i = 0; i < edgePointers.size(); ++i) {
            edgePointer->resetAngle();
            edgePointer->calcAngle();
            excess += edgePointer->getAngle();
            edgePointer = edgePointer->next;
        }
        excess -= (edgePointers.size()-2)*PI;
        if (fabs(excess) > 1.0 || excess <= 0.0)
            REPORT_DEBUG;
        excess = excess*Sphere::radius2;
        assert(excess == area);
#endif
    } else
        area = 0.0;
    if (isAreaSet) {
        this->area.save();
    }
    this->area.setNew(area);
    if (!isAreaSet) {
        this->area.save();
        isAreaSet = true;
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
        double mass = tracers[i].getMass()/edgePointers.size();
        EdgePointer *edgePointer = edgePointers.front();
        for (int j = 0; j < edgePointers.size(); ++j) {
            edgePointer->getPolygon(OrientRight)->tracers[i].addMass(mass);
            edgePointer = edgePointer->next;
        }
    }
}

void Polygon::handoverTracers(Polygon *polygon, double percent)
{
    for (int i = 0; i < tracers.size(); ++i) {
        double mass = tracers[i].getMass()*percent;
        polygon->tracers[i].addMass(mass);
    }
}
#endif

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