#ifndef CommonTasks_h
#define CommonTasks_h

#include <list>

class EdgePointer;

class CommonTasks
{
public:
    CommonTasks() {}
    virtual ~CommonTasks() {}

    enum TaskType {
        UpdateAngle
    };

    static void resetTasks();
    static int getTaskNumber(TaskType);
    static void recordTask(TaskType, EdgePointer *);
    static void deleteTask(TaskType, EdgePointer *);
    static void doTask(TaskType);
    static void dumpTask(TaskType);

private:
    static std::list<EdgePointer *> needUpdateAngles;
};

#endif
