#include "Polygon.h"
#include "Constants.h"
#include "Sphere.h"
#include "TimeManager.h"

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
        cout << "Angles:" << endl;
        edgePointer = edgePointers.front();
        for (int i = 0; i < edgePointers.size(); ++i) {
            cout << edgePointer->getAngle(OldTimeLevel)*Rad2Deg << "  " <<
            edgePointer->getAngle(NewTimeLevel)*Rad2Deg << "  " <<
            (edgePointer->getAngle(NewTimeLevel)-
             edgePointer->getAngle(OldTimeLevel))*Rad2Deg << endl;
            edgePointer = edgePointer->next;
        }
        cout << "Vertices:" << endl;
        edgePointer = edgePointers.front();
        for (int i = 0; i < edgePointers.size(); ++i) {
            Vertex *vertex = edgePointer->getEndPoint(FirstPoint);
            cout << vertex->getCoordinate().getLon() << "   " <<
                    vertex->getCoordinate().getLat() << endl;
            edgePointer = edgePointer->next;
        }
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
    static const double maxRatio = 0.1;
    double ratio = fabs(this->area.getOld()-this->area.getNew())/this->area.getOld();
    if (ratio > maxRatio) {
        cout << "Polygon ID: " << getID() << endl;
        cout << "Angles:" << endl;
        edgePointer = edgePointers.front();
        for (int i = 0; i < edgePointers.size(); ++i) {
            cout << edgePointer->getAngle(OldTimeLevel)*Rad2Deg << "  " <<
            edgePointer->getAngle(NewTimeLevel)*Rad2Deg << "  " <<
            (edgePointer->getAngle(NewTimeLevel)-
             edgePointer->getAngle(OldTimeLevel))*Rad2Deg << endl;
            edgePointer = edgePointer->next;
        }
        cout << "Vertices:" << endl;
        edgePointer = edgePointers.front();
        for (int i = 0; i < edgePointers.size(); ++i) {
            Vertex *vertex = edgePointer->getEndPoint(FirstPoint);
            cout << vertex->getCoordinate().getLon() << "   " <<
            vertex->getCoordinate().getLat() << endl;
            edgePointer = edgePointer->next;
        }
        cout << "Old area: ";
        cout << setw(20) << setprecision(15) << this->area.getOld() << endl;
        cout << "New area: ";
        cout << setw(20) << setprecision(15) << this->area.getNew() << endl;
        char message[100];
        sprintf(message, "Change of polygon area exceeds %f%% (%f%%)!", maxRatio*100.0, ratio*100.0);
        REPORT_ERROR(message);
    }
#endif
}

void Polygon::dump() const
{
    EdgePointer *edgePointer = edgePointers.front();
    for (int i = 0; i < edgePointers.size(); ++i) {
        edgePointer->getEndPoint(FirstPoint)->getCoordinate().dump();
        edgePointer = edgePointer->next;
    }
}