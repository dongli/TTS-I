#include "Constants.h"
#include "ConfigTools.h"
#include "Sphere.h"
#include "TimeManager.h"
#include "MeshManager.h"
#include "MeshAdaptor.h"
#include "FlowManager.h"
#include "StaticVortices.h"
#include "TTS.h"

int main(int argc, char **argv)
{
    MeshManager meshManager;
    MeshAdaptor meshAdaptor;
    FlowManager flowManager;
    TracerManager tracerManager;
    StaticVortices testCase;
    TTS tts;
    char fileName[30], filePattern[50] = "sq_360x180_%3.3d.nc";
    // -------------------------------------------------------------------------
    ConfigTools::parse("tts_square_config");
    TimeManager::setClock(1800.0);
    TimeManager::setEndStep(576);
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
    while (!TimeManager::isFinished()) {
        tts.advect(meshManager, meshAdaptor, flowManager, tracerManager);
        TimeManager::advance();
        testCase.calcVelocityField(flowManager);
        sprintf(fileName, filePattern, TimeManager::getSteps());
        tracerManager.output(fileName);
    }
}
