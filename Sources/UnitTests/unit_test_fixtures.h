#include "MeshManager.h"
#include "FlowManager.h"
#include "MovingVortices.h"
#include "SolidRotation.h"
#include "StaticVortices.h"

struct RLLMesh2D {
    RLLMesh2D() {
        lon = NULL;
        lat = NULL;
        RLLMesh2D_create(360, 180);    
    }
    ~RLLMesh2D() {
        delete [] lon;
        delete [] lat;
    }

    void RLLMesh2D_create(int numLon, int numLat) {
        this->numLon = numLon;
        this->numLat = numLat;

        double dlon = PI2/numLon;
        double dlat = PI/(numLat+1);

        if (lon != NULL) delete [] lon;
        if (lat != NULL) delete [] lat;
        
        lon = new double[numLon];
        lat = new double[numLat];
    
        for (int i = 0; i < numLon; ++i) {
            lon[i] = i*dlon;
        }
        for (int j = 0; j < numLat; ++j) {
            lat[j] = PI*0.5-(j+1)*dlat;
        }
    }

    int numLon, numLat;
    double *lon, *lat;
};

struct VelocityField2D {
    VelocityField2D() {
        meshManager.construct(rll.numLon, rll.numLat, rll.lon, rll.lat);
        flowManager.construct(meshManager);
    }
    ~VelocityField2D() {}

    void VelocityField2D_create(int numLon, int numLat) {
        rll.RLLMesh2D_create(numLon, numLat);
        meshManager.construct(rll.numLon, rll.numLat, rll.lon, rll.lat);
        flowManager.construct(meshManager);
    }

    RLLMesh2D rll;
    MeshManager meshManager;
    FlowManager flowManager;
};
