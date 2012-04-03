#include "ConfigTools.h"
#include "Sphere.h"
#include "TimeManager.h"
#include "GAMILReader.h"
#include "PolygonRezoner.h"
#include "TTS.h"

int main(int argc, char **argv)
{
    TimeManager timeManager;
    GAMILReader gamilReader;
    TracerManager tracerManager;
    TTS tts;
    char fileName[30], filePattern[50] = "gamil_2562_%5.5d.nc";
    // -------------------------------------------------------------------------
    ConfigTools::parse("tts_gamil_config");
    Sphere::setRadius(6371.299e3);
    // -------------------------------------------------------------------------
    tracerManager.init(argv[1]);
    tts.init();
    // -------------------------------------------------------------------------
    gamilReader.init(argv[2], "tts.gamil.suv.*.nc");
    gamilReader.getTracerField(tracerManager);
#ifdef TTS_REZONE
    PolygonRezoner::rezone(gamilReader.meshManager, gamilReader.meshAdaptor,
                           gamilReader.flowManager, tracerManager);
#endif
    gamilReader.getVelocityField();
#ifdef TTS_OUTPUT
    sprintf(fileName, filePattern, timeManager.getSteps());
    tracerManager.output(fileName);
#endif
    // -------------------------------------------------------------------------
    while (!timeManager.isFinished()) {
        tts.advect(gamilReader.meshManager, gamilReader.meshAdaptor,
                   gamilReader.flowManager, tracerManager);
        timeManager.advance();
        gamilReader.getVelocityField();
#ifdef TTS_OUTPUT
        sprintf(fileName, filePattern, timeManager.getSteps());
        tracerManager.output(fileName);
#endif
    }
}
