#ifndef MeshAdaptor_h
#define MeshAdaptor_h

#include <blitz/array.h>
#include <string>
#include <list>

using blitz::Array;
using std::string;
using std::list;

#include "Vector.h"
#include "Coordinate.h"
#include "Field.h"
#include "TracerManager.h"

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
    void adapt(const TracerManager &tracerManager,
               const MeshManager &meshManager);

    //! \brief Remap the quantities associated with fixed mesh to tracer.
    void remap(const string &tracerName, const Field &q,
               TracerManager &tracerManager);
    void remap(const string &tracerName, TracerManager &tracerManager);

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

    typedef struct {
        Polygon *polygon;
        double area, totalArea;
    } OverlapArea;

    void recordOverlapArea(double cellArea, int I, int J,
                           Bnd from, Bnd to, int bndDiff,
                           Polygon *polygon, double area, double &totalArea,
                           list<OverlapArea *> &overlapAreas);
    void recordOverlapArea(double cellArea, int I, int J,
                           Polygon *polygon, double area, double &totalArea,
                           list<OverlapArea *> &overlapAreas);

    Array<list<OverlapArea>, 3> overlapAreaList;
};

#endif
