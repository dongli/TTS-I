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

//#define MOVINGVORTICES_TESTCASE
//#define TESTCASE_CALC_TRUE_SOLUTION
#define DEFORMATION_TESTCASE
//#define STATICVORTICES_TESTCASE
//#define SOLIDROTATION_TESTCASE

int main(int argc, char **argv)
{
    MeshManager meshManager;
    MeshAdaptor meshAdaptor;
    FlowManager flowManager;
    TracerManager tracerManager;
    TTS tts;
    // -------------------------------------------------------------------------
#ifdef MOVINGVORTICES_TESTCASE
    MovingVortices testCase;
#ifdef TESTCASE_CALC_TRUE_SOLUTION
    char fileName[30], filePattern[50] = "mv_true_360x180_40962_576_%3.3d.nc";
#else
    char fileName[30], filePattern[50] = "mv_360x180_2562_576_%3.3d.nc";
#endif
    ConfigTools::parse("tts_mv_config");
    TimeManager::setClock(1800.0);
    TimeManager::setEndStep(576);
#endif
    // -------------------------------------------------------------------------
#ifdef DEFORMATION_TESTCASE
    Deformation testCase(Deformation::Case4, Deformation::SlottedCylinders);
    char fileName[30], filePattern[50] = "sc_360x180_2562_600_%3.3d.nc";
    ConfigTools::parse("tts_df_config");
    TimeManager::setClock(5.0/600.0);
    TimeManager::setEndStep(600);
#endif
    // -------------------------------------------------------------------------
#ifdef STATICVORTICES_TESTCASE
    StaticVortices testCase;
    char fileName[30], filePattern[50] = "square_%4.4d.nc";
    ConfigTools::parse("tts_sv_config");
    TimeManager::setClock(1800.0);
    TimeManager::setEndStep(1152);
#endif
    // -------------------------------------------------------------------------
#ifdef SOLIDROTATION_TESTCASE
    SolidRotation testCase;
    char fileName[30], filePattern[50] = "sr_360x180_360x180_%4.4d.nc";
    ConfigTools::parse("tts_sr_config");
    TimeManager::setClock(1800.0);
    TimeManager::setEndStep(100);
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
    // -------------------------------------------------------------------------
    tracerManager.init(argv[1]);
    tts.init();
    // -------------------------------------------------------------------------
    testCase.calcInitCond(meshManager, meshAdaptor, tracerManager);
#ifdef TESTCASE_CALC_TRUE_SOLUTION
    testCase.calcSolution(meshManager, meshAdaptor, tracerManager);
#endif
    sprintf(fileName, filePattern, TimeManager::getSteps());
#ifdef TTS_OUTPUT
    tracerManager.output(fileName);
#endif
    // -------------------------------------------------------------------------
    while (!TimeManager::isFinished()) {
        tts.advect(meshManager, meshAdaptor, flowManager, tracerManager);
#ifdef TESTCASE_CALC_TRUE_SOLUTION
        testCase.calcSolution(meshManager, meshAdaptor, tracerManager);
#endif
        TimeManager::advance();
        testCase.calcVelocityField(flowManager);
#ifdef TTS_OUTPUT
        sprintf(fileName, filePattern, TimeManager::getSteps());
        tracerManager.output(fileName);
#endif
    }
}
