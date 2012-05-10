#include "Constants.hpp"
#include "ConfigTools.hpp"
#include "Sphere.hpp"
#include "TimeManager.hpp"
#include "MeshManager.hpp"
#include "FlowManager.hpp"
#include "SolidRotation.hpp"
#include "MovingVortices.hpp"
#include "StaticVortices.hpp"
#include "Deformation.hpp"
#include "TTS.hpp"
#include "MeshAdaptor.hpp"
#include "PolygonRezoner.hpp"
#include "CppHelper.hpp"

#define MOVINGVORTICES_TESTCASE
//#define TESTCASE_CALC_TRUE_SOLUTION
//#define DEFORMATION_TESTCASE
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
    char filePattern[50];
    ConfigTools::read("output_file_pattern", filePattern);
    tracerManager.output(to_string(TimeManager::getSteps(), filePattern));
#endif
    // -------------------------------------------------------------------------
    while (!TimeManager::isFinished()) {
        TimeManager::advance();
        testCase.calcVelocityField(flowManager);
        tts.advect(meshManager, meshAdaptor, flowManager, tracerManager);
#ifdef TESTCASE_CALC_TRUE_SOLUTION
        testCase.calcSolution(meshManager, meshAdaptor, tracerManager);
#endif
#ifdef TTS_OUTPUT
        tracerManager.output(to_string(TimeManager::getSteps(), filePattern));
#endif
    }
}
