#ifndef Tracer_h
#define Tracer_h

#include <string>

using std::string;

#include "List.h"
#include "Parcel.h"
#include "PolygonManager.h"

class Tracer : public ListElement<Tracer>
{
public:
    Tracer();
    ~Tracer();

    void reinit();

    void init(const string &name, const string &long_name, const string &unit,
              const MeshManager &, const PolygonManager &);

    const string &getName() const { return name; }

    void update();
    void remap();

    bool operator==(const Tracer &);

private:
    string name;
    Field density;
    List<Parcel> parcels;
};

#endif
