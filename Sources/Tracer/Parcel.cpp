#include "Parcel.h"

Parcel::Parcel()
{
    polygon = NULL;
}

Parcel::~Parcel()
{
}

void Parcel::reinit()
{
    polygon = NULL;
}

void Parcel::linkPolygon(Polygon *polygon)
{
#ifdef DEBUG
    assert(this->polygon == NULL);
#endif
    this->polygon = polygon;
}

void Parcel::attachTracer()
{
    this->mass.push_back(-999.0);
    this->density.push_back(-999.0);
}

void Parcel::update()
{
    for (int i = 0; i < mass.size(); ++i) {
        density[i] = mass[i]/polygon->getArea();
    }
}