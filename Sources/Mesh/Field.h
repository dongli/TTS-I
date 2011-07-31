#ifndef _Field_h_
#define _Field_h_

#include "RLLMesh.h"
#include "Layers.h"
#include "MultiTimeLevel.h"
#include "Coordinate.h"
#include "Location.h"
#include <string>
#include <blitz/array.h>

using std::string;
using blitz::Array;
using blitz::Range;

class Field
{
public:
    Field();
    virtual ~Field();

    void construct(const RLLMesh &mesh);
    void construct(const RLLMesh &mesh, const Layers &layers);

    double interp(const Coordinate &, const Location &, TimeLevel) const;

    const RLLMesh *mesh;
    const Layers *layers;
    Array<MultiTimeLevel<double, 2>, 3> values;

    string name;
    string long_name;
    string unit;
};

#endif
