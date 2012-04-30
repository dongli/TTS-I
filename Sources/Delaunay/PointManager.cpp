#include "PointManager.hpp"

PointManager::PointManager()
{
}

PointManager::~PointManager()
{
    PointPointer *point = points.front();
    for (int i = 0; i < points.size(); ++i) {
        delete point->ptr;
        point = point->next;
    }
}

void PointManager::init(int numPoint, double *lon, double *lat)
{
    points.create(numPoint);
    points.setName("Points");
    PointPointer *point = points.front();
    for (int i = 0; i < numPoint; ++i) {
        point->ptr = new Point;
        point->ptr->setCoordinate(lon[i], lat[i]);
        point = point->next;
    }
}
