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
    Deformation testCase(Deformation::Case4, Deformation::GaussianHills);
//    MovingVortices testCase;
    TTS tts;

//    char fileName[30], filePattern[50] = "gh_360x180_2562_600_%3.3d.nc";
//    char fileName[30], filePattern[50] = "ch_240x120_240x120_600_%3.3d.nc";
//    char fileName[30], filePattern[50] = "ch_128x60_128x60_600_%3.3d.nc";
//    char fileName[30], filePattern[50] = "mv_%3.3d.nc";

    // -------------------------------------------------------------------------
    timeManager.setClock(5.0/600);
    timeManager.setEndStep(600);
//    timeManager.setClock(1800.0);
//    timeManager.setEndStep(576);

    // -------------------------------------------------------------------------
    int numLon = 360, numLat = 180;
//    int numLon = 128, numLat = 60;
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
//    flowManager.output("flow.nc");

    // -------------------------------------------------------------------------
    tracerManager.init(argv[1]);

    // -------------------------------------------------------------------------
//    testCase.calcInitCond(meshManager, meshAdaptor, tracerManager);
//    sprintf(fileName, filePattern, timeManager.getSteps());
//    tracerManager.output(fileName);

    // -------------------------------------------------------------------------
    while (!timeManager.isFinished()) {
        tts.advect(meshManager, meshAdaptor, flowManager, tracerManager);
        timeManager.advance();
        testCase.calcVelocityField(flowManager);
//        sprintf(fileName, filePattern, timeManager.getSteps());
//        tracerManager.output(fileName);
//        flowManager.output("flow.nc");
    }
}
