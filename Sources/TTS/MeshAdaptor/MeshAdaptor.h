#ifndef MeshAdaptor_h
#define MeshAdaptor_h

#include <blitz/array.h>
#include <string>

using blitz::Array;

class PolygonManager;
class MeshManager;

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
    // store the remapping coefficients
    Array<Array<double, 1>, 3> overlapAreas;
};

#endif
