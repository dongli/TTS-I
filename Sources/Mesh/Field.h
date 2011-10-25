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

    void init(const RLLMesh &meshCnt);
    void init(const RLLMesh &meshCnt, const Layers &layers);
    void init(const RLLMesh &meshCnt, const RLLMesh &meshBnd);
    void init(const RLLMesh &meshCnt, const RLLMesh &meshBnd, const Layers &layers);

    enum MeshType { Center = 0, Bound = 1 };

    const RLLMesh &getMesh(MeshType type = Center) const { return *mesh[type]; }

    double interp(const Coordinate &, const Location &, TimeLevel) const;

    const RLLMesh *mesh[2];
    const Layers *layers;
    Array<MultiTimeLevel<double, 2>, 3> values;

    string name;
    string long_name;
    string unit;
};

#endif
