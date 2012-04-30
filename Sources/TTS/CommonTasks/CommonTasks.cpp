#include "CommonTasks.hpp"
#include "ReportMacros.hpp"
#include "Edge.hpp"
#include "ApproachDetector.hpp"
#include "PolygonManager.hpp"

using namespace ApproachDetector;

std::set<EdgePointer *> CommonTasks::needUpdateAngles;
std::set<Edge *> CommonTasks::needRemoveEdges;
std::set<Vertex *> CommonTasks::needRemoveVertices;

void CommonTasks::resetTasks()
{
    needUpdateAngles.clear();
}

int CommonTasks::getTaskNumber(TaskType type)
{
    if (type == UpdateAngle)
        return static_cast<int>(needUpdateAngles.size());
    else if (type == RemoveObject)
        return static_cast<int>(needRemoveEdges.size()+
                                needRemoveVertices.size());
    else
        REPORT_ERROR("Unknown task type!");
}

void CommonTasks::recordTask(TaskType type, EdgePointer *edgePointer)
{
    if (type == UpdateAngle) {
        edgePointer->resetAngle();
        needUpdateAngles.insert(edgePointer);
    }
}

void CommonTasks::recordTask(TaskType type, Edge *edge)
{
    if (type == RemoveObject) {
        needRemoveEdges.insert(edge);
    }
}

void CommonTasks::recordTask(TaskType type, Vertex *vertex)
{
    if (type == RemoveObject) {
        needRemoveVertices.insert(vertex);
    }
}

void CommonTasks::deleteTask(TaskType type, EdgePointer *edgePointer)
{
    if (type == UpdateAngle) {
        needUpdateAngles.erase(edgePointer);
    }
}

void CommonTasks::deleteTask(TaskType type, Edge *edge)
{
    if (type == RemoveObject) {
        needRemoveEdges.erase(edge);
    }
}

void CommonTasks::deleteTask(TaskType type, Vertex *vertex)
{
    if (type == RemoveObject) {
        needRemoveVertices.erase(vertex);
    }
}

void CommonTasks::doTask(TaskType type)
{
    if (type == UpdateAngle) {
        std::set<EdgePointer *>::iterator it;
        for (it = needUpdateAngles.begin();
             it != needUpdateAngles.end(); ++it) {
#ifdef DEBUG
            assert((*it)->isAngleSet == false);
#endif
            (*it)->calcAngle();
        }
        needUpdateAngles.clear();
    } else {
        REPORT_ERROR("Unknown task type!");
    }
}

void CommonTasks::doTask(TaskType type, PolygonManager &polygonManager)
{
    if (type == RemoveObject) {
        std::set<Edge *>::iterator it1;
        for (it1 = needRemoveEdges.begin();
             it1 != needRemoveEdges.end(); ++it1) {
            polygonManager.edges.remove(*it1);
        }
        needRemoveEdges.clear();
        std::set<Vertex *>::iterator it2;
        for (it2 = needRemoveVertices.begin();
             it2 != needRemoveVertices.end(); ++it2) {
            polygonManager.vertices.remove(*it2);
        }
        needRemoveVertices.clear();
    } else {
        REPORT_ERROR("Unknown task type!");
    }
}

void CommonTasks::dumpTask(TaskType type)
{
    if (type == UpdateAngle) {
        cout << setw(8) << "Edge ID";
        cout << setw(8) << "Orient";
        cout << setw(8) << "ID";
        cout << setw(20) << "First point ID";
        cout << setw(20) << "Second point ID" << endl;
        std::set<EdgePointer *>::const_iterator it;
        for (it = needUpdateAngles.begin();
             it != needUpdateAngles.end(); ++it) {
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
    } else if (type == RemoveObject) {
        cout << "To be removed edges:" << endl;
        std::set<Edge *>::const_iterator it1;
        for (it1 = needRemoveEdges.begin();
             it1 != needRemoveEdges.end(); ++it1) {
            cout << setw(8) << (*it1)->getID() << endl;
        }
        cout << "To be removed vertices:" << endl;
        std::set<Vertex *>::const_iterator it2;
        for (it2 = needRemoveVertices.begin();
             it2 != needRemoveVertices.end(); ++it2) {
            cout << setw(8) << (*it2)->getID() << endl;
        }
    } else {
        REPORT_ERROR("Unknown task type!");
    }
}