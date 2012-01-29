#include "Constants.h"
#include "ConfigTools.h"
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

#define MOVINGVORTICES_TESTCASE

int main(int argc, char **argv)
{
    MeshManager meshManager;
    MeshAdaptor meshAdaptor;
    FlowManager flowManager;
    TracerManager tracerManager;
    TTS tts;
#ifdef MOVINGVORTICES_TESTCASE
    MovingVortices testCase;
    char fileName[30], filePattern[50] = "mv_360x180_10242_576_%3.3d.nc";
#endif
#ifdef DEFORMATION_TESTCASE
    Deformation testCase(Deformation::Case4, Deformation::GaussianHills);
    char fileName[30], filePattern[50] = "gh_360x180_2562_600_%3.3d.nc";
#endif
    // -------------------------------------------------------------------------
#ifdef MOVINGVORTICES_TESTCASE
    ConfigTools::parse("tts_mv_config");
    TimeManager::setClock(1800.0);
    TimeManager::setEndStep(576);
#endif
#ifdef DEFORMATION_TESTCASE
    ConfigTools::parse("tts_df_config");
    TimeManager::setClock(5.0/600.0);
    TimeManager::setEndStep(600);
#endif
    // -------------------------------------------------------------------------
    int numLon = 360, numLat = 180;
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
    flowManager.output("flow.nc");
    // -------------------------------------------------------------------------
    tracerManager.init(argv[1]);
    tts.init();
    // -------------------------------------------------------------------------
    testCase.calcInitCond(meshManager, meshAdaptor, tracerManager);
    sprintf(fileName, filePattern, TimeManager::getSteps());
#ifdef TTS_OUTPUT
    tracerManager.output(fileName);
#endif
    // -------------------------------------------------------------------------
    while (!TimeManager::isFinished()) {
        tts.advect(meshManager, meshAdaptor, flowManager, tracerManager);
        TimeManager::advance();
        testCase.calcVelocityField(flowManager);
#ifdef TTS_OUTPUT
        sprintf(fileName, filePattern, TimeManager::getSteps());
        tracerManager.output(fileName);
#endif
    }
}
