#ifndef _RLLMesh_h_
#define _RLLMesh_h_

#include <blitz/array.h>

using blitz::Array;

class RLLMesh
{
public:
	enum MeshType {
	    Full = 0, LonHalf = 1, LatHalf = 2, BothHalf = 3
	};

    RLLMesh();
    RLLMesh(MeshType type, int numLon, int numLat, double *lon, double *lat);
    virtual ~RLLMesh();

    void init(MeshType type, int numLon, int numLat, double *lon, double *lat);

    int getNumLon() const { return static_cast<int>(lon.size()); }
    int getNumLat() const { return static_cast<int>(lat.size()); }

    void dump() const;

    MeshType type;
    Array<double, 1> lon, lat;
    Array<double, 1> cosLat;
    Array<double, 2> area;

	double dlon;
    Array<double, 1> dlat;

    bool isConstructed;
};

#endif
