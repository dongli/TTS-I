#include "Constants.h"
#include "Sphere.h"
#include "TimeManager.h"
#include "MeshManager.h"
#include "FlowManager.h"
#include "SolidRotation.h"
#include "MovingVortices.h"
#include "StaticVortices.h"
#include "Deformation.h"
#include "TTS.h"
#include "MeshAdaptor.h"

int main(int argc, char **argv)
{
    TimeManager timeManager;
    MeshManager meshManager;
    MeshAdaptor meshAdaptor;
    FlowManager flowManager;
    TracerManager tracerManager;
    //SolidRotation testCase;
    //MovingVortices testCase;
    //StaticVortices testCase;
    Deformation testCase;
    TTS tts;

    // -------------------------------------------------------------------------
    timeManager.setClock(5.0/120);
    timeManager.setEndStep(120);

    // -------------------------------------------------------------------------
    tracerManager.init(argv[1]);

    // -------------------------------------------------------------------------
    int numLon = 128, numLat = 60;
    double dlon = PI2/numLon;
    double dlat = PI/(numLat+1);
    double lon[numLon], lat[numLat];
    for (int i = 0; i < numLon; ++i)
        lon[i] = i*dlon;
    for (int j = 0; j < numLat; ++j)
        lat[j] = PI05-(j+1)*dlat;

    meshManager.init(numLon, numLat, lon, lat);
    meshAdaptor.init(meshManager);
    flowManager.init(meshManager);
    testCase.calcVelocityField(flowManager);
    //flowManager.output("flow.nc");

    // -------------------------------------------------------------------------
    while (!timeManager.isFinished()) {
        tts.advect(meshManager, meshAdaptor, flowManager, tracerManager);
        timeManager.advance();
        testCase.calcVelocityField(flowManager);
        char fileName[30];
        sprintf(fileName, "tts_test%5.5d.nc", timeManager.getSteps());
        tracerManager.output(fileName);
        //flowManager.output("flow.nc");
    }
}
