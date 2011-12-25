#include "CommonTasks.h"
#include "ReportMacros.h"
#include "Edge.h"
#include "ApproachDetector.h"

using namespace ApproachDetector;

std::list<EdgePointer *> CommonTasks::needUpdateAngles;

void CommonTasks::resetTasks()
{
    needUpdateAngles.clear();
}

int CommonTasks::getTaskNumber(TaskType type)
{
    if (type == UpdateAngle)
        return static_cast<int>(needUpdateAngles.size());
    else
        REPORT_ERROR("Unknown task type!");
}

void CommonTasks::recordTask(TaskType type, EdgePointer *edgePointer)
{
    if (type == UpdateAngle) {
        if (find(needUpdateAngles.begin(), needUpdateAngles.end(), edgePointer)
            == needUpdateAngles.end()) {
            edgePointer->resetAngle();
            needUpdateAngles.push_back(edgePointer);
        }
    }
}

void CommonTasks::deleteTask(TaskType type, EdgePointer *edgePointer)
{
    if (type == UpdateAngle) {
#ifdef DEBUG
        if (find(needUpdateAngles.begin(), needUpdateAngles.end(), edgePointer)
            == needUpdateAngles.end()) {
            //dumpTask(type);
            REPORT_WARNING("The edge pointer does not contained in the list.")
        }
#endif
        needUpdateAngles.remove(edgePointer);
    }
}

void CommonTasks::doTask(TaskType type)
{
    if (type == UpdateAngle) {
        std::list<EdgePointer *>::iterator it;
        for (it = needUpdateAngles.begin();
             it != needUpdateAngles.end(); ++it) {
#ifdef DEBUG
            assert((*it)->isAngleSet == false);
#endif
            (*it)->calcAngle();
        }
        needUpdateAngles.clear();
    }
}

void CommonTasks::dumpTask(TaskType type)
{
    std::list<EdgePointer *>::const_iterator it;
    it = needUpdateAngles.begin();
    cout << setw(8) << "Edge ID";
    cout << setw(8) << "Orient";
    cout << setw(8) << "ID";
    cout << setw(20) << "First point ID";
    cout << setw(20) << "Second point ID" << endl;
    for (; it != needUpdateAngles.end(); ++it) {
        cout << setw(8) << (*it)->edge->getID();
        if ((*it)->orient == OrientLeft)
            cout << setw(8) << "left";
        else
            cout << setw(8) << "right";
        cout << setw(8) << (*it)->getID();
        cout << setw(20) << (*it)->getEndPoint(FirstPoint)->getID();
        cout << setw(20) << (*it)->getEndPoint(SecondPoint)->getID();
        cout << endl;
    }
}