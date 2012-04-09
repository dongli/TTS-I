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

typedef struct {
    Polygon *polygon;
    double area, totalArea;
} OverlapArea;

class MeshAdaptor
{
public:
    MeshAdaptor();
    virtual ~MeshAdaptor();

    /*
     * Function:
     *   init
     * Purpose:
     *   Initialize mesh adaptor.
     */
    void init(const MeshManager &meshManager);

    /*
     * Function:
     *   adapt
     * Purpose:
     *   Adapt PolygonManager and MeshManager for each other.
     *   The overlapping areas are calculated.
     */
    void adapt(const TracerManager &tracerManager,
               const MeshManager &meshManager);

    /*
     * Function:
     *   remap
     * Purpose:
     *   Remap the quantities associated with fixed mesh to tracer.
     */
    void remap(const string &tracerName, const Field &q,
               TracerManager &tracerManager);
    void remap(const string &tracerName, TracerManager &tracerManager);

    /*
     * Function:
     *   getOverlapAreaList
     * Purpose:
     *   Return overlap area list (for generating SCVT density function) 
     */
    const Array<list<OverlapArea>, 3> &getOverlapAreaList() const {
        return overlapAreaList;
    }

private:
    enum Bnd {
        WestBnd, SouthBnd, EastBnd, NorthBnd, NullBnd
    };

    static double calcCorrectArea(const Coordinate &x1, const Coordinate &x2,
                                  const Vector &normVector, int signFlag);
    double calcOverlapArea(int I, int J, Bnd from, Bnd to,
                           int &bndDiff, bool &isTolerated,
                           double lonBnd1, double lonBnd2,
                           double latBnd1, double latBnd2,
                           Coordinate x0, EdgePointer *edgePointer0,
                           Coordinate x1, EdgePointer *edgePointer1);
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
