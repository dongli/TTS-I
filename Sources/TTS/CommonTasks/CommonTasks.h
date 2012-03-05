#ifndef CommonTasks_h
#define CommonTasks_h

#include <set>

class EdgePointer;
class Edge;
class Vertex;
class PolygonManager;

class CommonTasks
{
public:
    CommonTasks() {}
    virtual ~CommonTasks() {}

    enum TaskType {
        UpdateAngle, RemoveObject
    };

    static void resetTasks();
    static int getTaskNumber(TaskType);
    static void recordTask(TaskType, EdgePointer *);
    static void recordTask(TaskType, Edge *);
    static void recordTask(TaskType, Vertex *);
    static void deleteTask(TaskType, EdgePointer *);
    static void deleteTask(TaskType, Edge *);
    static void deleteTask(TaskType, Vertex *);
    static void doTask(TaskType);
    static void doTask(TaskType, PolygonManager &);
    static void dumpTask(TaskType);

private:
    static std::set<EdgePointer *> needUpdateAngles;
    static std::set<Edge *> needRemoveEdges;
    static std::set<Vertex *> needRemoveVertices;
};

#endif
