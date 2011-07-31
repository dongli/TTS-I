#include "Constants.h"
#include "Sphere.h"
#include "TimeManager.h"
#include "MeshManager.h"
#include "FlowManager.h"
#include "SolidRotation.h"
#include "MovingVortices.h"
#include "StaticVortices.h"
#include "TTS.h"

int main(int argc, char **argv)
{
    TimeManager timeManager;
    MeshManager meshManager;
    FlowManager flowManager;
    ParcelManager parcelManager;
    //SolidRotation testCase;
    MovingVortices testCase;
    //StaticVortices testCase;
    TTS tts;

    // -------------------------------------------------------------------------
    timeManager.setClock(1800.0);
    timeManager.setEndStep(1152);
    
    // -------------------------------------------------------------------------
    parcelManager.construct(argv[1]);

    // -------------------------------------------------------------------------
    int numLon = 128, numLat = 60;
    double dlon = PI2/numLon;
    double dlat = PI/(numLat+1);
    double lon[numLon], lat[numLat];
    for (int i = 0; i < numLon; ++i)
        lon[i] = i*dlon;
    for (int j = 0; j < numLat; ++j)
        lat[j] = PI05-(j+1)*dlat;

    meshManager.construct(numLon, numLat, lon, lat);
    flowManager.construct(meshManager);
    testCase.calcVelocityField(flowManager);
    //flowManager.output("flow.nc");

    // -------------------------------------------------------------------------
    while (!timeManager.isFinished()) {
        tts.advect(meshManager, flowManager, parcelManager);
        timeManager.advance();
        testCase.calcVelocityField(flowManager);
        char fileName[30];
        sprintf(fileName, "tts_test%5.5d.nc", timeManager.getSteps());
        parcelManager.output(fileName);
        //flowManager.output("flow.nc");
    }
}
