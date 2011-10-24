#ifndef RLLMesh_h
#define RLLMesh_h

#include <string>
#include <blitz/array.h>

using std::string;
using blitz::Array;

enum MeshType {
    Full = 0, LonHalf = 1, LatHalf = 2, BothHalf = 3
};

struct MeshSpec {
    MeshType type;
    bool isWithPoles;
    bool isAreaFit;
};

class RLLMesh
{
public:
    RLLMesh();
    RLLMesh(MeshSpec spec, int numLon, int numLat, double *lon, double *lat);
    virtual ~RLLMesh();

    void init(MeshSpec spec, int numLon, int numLat, double *lon, double *lat);

    int getNumLon() const { return static_cast<int>(lon.size()); }
    int getNumLat() const { return static_cast<int>(lat.size()); }

    void dump() const;
    void output(const string &fileName) const;

    MeshSpec spec;
    Array<double, 1> lon, lat;
    Array<double, 1> cosLat;
    Array<double, 2> area;

	double dlon;
    Array<double, 1> dlat;

    bool isConstructed;
};

#endif
