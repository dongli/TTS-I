#ifndef Field_h
#define Field_h

#include "RLLMesh.hpp"
#include "Layers.hpp"
#include "MultiTimeLevel.hpp"
#include "Coordinate.hpp"
#include "Location.hpp"
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

    /*
     * Operator:
     *   ()
     * Purpose:
     *   These function operators are used to hide the error-prone zonal
     *   boundary conditions. There are two meridinal ghost columns with
     *   internal index 0 and Nlon-1, but with these operators, users can
     *   normally access the real elements with longitude index [0,Nlon-2].
     *
     *   * Nlon = mesh[Center].getNumLon().
     */
    MultiTimeLevel<double, 2> &operator()(int i, int j, int k = 0);
    const MultiTimeLevel<double, 2> &operator()(int i, int j, int k = 0) const;

    const RLLMesh *mesh[2];
    const Layers *layers;
    Array<MultiTimeLevel<double, 2>, 3> values;

    string name;
    string long_name;
    string unit;
};

#endif
