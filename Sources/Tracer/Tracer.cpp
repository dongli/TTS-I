#include "Tracer.h"

Tracer::Tracer()
{
}

Tracer::~Tracer()
{
    parcels.destroy();
}

void Tracer::reinit()
{
    parcels.recycle();
}

void Tracer::init(const string &name, const string &long_name,
                  const string &unit, const MeshManager &meshManager,
                  const PolygonManager &polygonManager)
{
    // -------------------------------------------------------------------------
    density.name = name;
    density.long_name = long_name;
    density.unit = unit;
    if (meshManager.hasLayers())
        density.init(meshManager.getMesh(RLLMesh::Full),
                     meshManager.getLayers(Layers::Full));
    else
        density.init(meshManager.getMesh(RLLMesh::Full));
    // -------------------------------------------------------------------------
#ifdef DEBUG
    assert(parcels.size() == 0);
#endif
    parcels.create(polygonManager.polygons.size());
    Parcel *parcel = parcels.front();
    Polygon *polygon = polygonManager.polygons.front();
    for (int i = 0; i < parcels.size(); ++i) {
        parcel->linkPolygon(polygon);
        parcel = parcel->next;
        polygon = polygon->next;
    }
}

void Tracer::update()
{
    Parcel *parcel = parcels.front();
    for (int i = 0; i < parcels.size(); ++i) {
        parcel->update();
        parcel = parcel->next;
    }
}

void Tracer::remap()
{
    
}

bool Tracer::operator==(const Tracer &that)
{
    if (this->density.name != that.density.name)
        return false;
    else
        return true;
}