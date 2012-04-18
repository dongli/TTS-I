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
#include "PolygonRezoner.h"

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
    ConfigTools::parse("tts_mv_config");
    TimeManager::setClock(1800.0);
    TimeManager::setEndStep(576);
#endif
    // -------------------------------------------------------------------------
#ifdef DEFORMATION_TESTCASE
    ConfigTools::parse("tts_config");
    string caseID, initCondID;
    ConfigTools::read("case_id", caseID);
    ConfigTools::read("init_cond_id", initCondID);
    Deformation testCase(caseID, initCondID);
    TimeManager::setClock(5.0/600.0);
    TimeManager::setEndStep(600);
#endif
    // -------------------------------------------------------------------------
#ifdef STATICVORTICES_TESTCASE
    StaticVortices testCase;
    ConfigTools::parse("tts_sv_config");
    TimeManager::setClock(1800.0);
    TimeManager::setEndStep(1152);
#endif
    // -------------------------------------------------------------------------
#ifdef SOLIDROTATION_TESTCASE
    SolidRotation testCase;
    ConfigTools::parse("tts_sr_config");
    TimeManager::setClock(1800.0);
    TimeManager::setEndStep(100);
#endif
    char fileName[30], filePattern[50];
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
    tracerManager.init();
    tts.init();
    // -------------------------------------------------------------------------
    testCase.calcInitCond(meshManager, meshAdaptor, tracerManager);
#ifdef TTS_REZONE
    PolygonRezoner::rezone(meshManager, meshAdaptor, flowManager, tracerManager);
#endif
#ifdef TESTCASE_CALC_TRUE_SOLUTION
    testCase.calcSolution(meshManager, meshAdaptor, tracerManager);
#endif
#ifdef TTS_OUTPUT
    ConfigTools::read("output_file_pattern", filePattern);
    sprintf(fileName, filePattern, TimeManager::getSteps());
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
