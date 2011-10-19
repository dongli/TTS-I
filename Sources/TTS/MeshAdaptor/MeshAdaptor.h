#ifndef MeshAdaptor_h
#define MeshAdaptor_h

#include "Vector.h"
#include "Coordinate.h"

#include <blitz/array.h>
#include <string>
#include <list>

using blitz::Array;
using std::list;

class PolygonManager;
class MeshManager;
class Polygon;
class EdgePointer;

class MeshAdaptor
{
public:
    MeshAdaptor();
    virtual ~MeshAdaptor();

    //! \brief Initialize mesh adaptor.
    //! \param meshManager The mesh that will be associated with.
    //! \return none.
    void init(const MeshManager &meshManager);

    //! \brief Adapt PolygonManager and MeshManager for each other.
    //!        The overlapping areas are calculated.
    //! \param polygonManager The source polygons.
    //! \param meshManager The target mesh.
    //! \return none.
    void adapt(const PolygonManager &polygonManager,
               const MeshManager &meshManager);

    //! \brief Remap the quantities associated with polygons to fixed mesh.
    //! \param polygonManager The source polygons.
    //! \param meshManager The target mesh.
    //! \return none.
    void remap(const PolygonManager &polygonManager,
               const MeshManager &meshManager);

private:    
    enum Bnd {
        WestBnd, SouthBnd, EastBnd, NorthBnd, NullBnd
    };

    double calcCorrectArea(const Coordinate &x1, const Coordinate &x2,
                           const Vector &normVector, int signFlag);

    double calcOverlapArea(int I, int J, Bnd from, Bnd to, int &bndDiff,
                           double lonBnd1, double lonBnd2,
                           double latBnd1, double latBnd2,
                           const Coordinate &x0, EdgePointer *edgePointer0,
                           const Coordinate &x1, EdgePointer *edgePointer1);

    void recordOverlapArea(double cellArea, int I, int J,
                           Bnd from, Bnd to, int bndDiff,
                           Polygon *polygon, double area, double &totalArea);
    void recordOverlapArea(double cellArea, int I, int J,
                           Polygon *polygon, double area, double &totalArea);

    // store the remapping coefficients
    typedef struct {
        Polygon *polygon;
        double area;
        int numPart = 0;
        Bnd from[10], to[10];
        int bndDiff[10];
    } OverlapPolygon;
    Array<list<OverlapPolygon>, 3> overlapPolygons;
};

#endif
