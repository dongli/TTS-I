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
    // -------------------------------------------------------------------------
    char fileName[225], dirName[225], filePattern[50] = "gamil_2562_%5.5d.nc";
    // -------------------------------------------------------------------------
    ConfigTools::parse(argv[1]);
    Sphere::setRadius(6371.299e3);
    // -------------------------------------------------------------------------
    ConfigTools::read("parcel_polygon_file", fileName);
    tracerManager.init(fileName);
    tts.init();
    // -------------------------------------------------------------------------
    ConfigTools::read("gamil_data_root", dirName);
    ConfigTools::read("gamil_data_pattern", fileName);
    gamilReader.init(dirName, fileName);
    gamilReader.getTracerField(tracerManager);
#ifdef TTS_REZONE
    PolygonRezoner::rezone(gamilReader.meshManager, gamilReader.meshAdaptor,
                           gamilReader.flowManager, tracerManager);
#endif
    gamilReader.getVelocityField();
    gamilReader.checkVelocityField();
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
        gamilReader.checkVelocityField();
#ifdef TTS_OUTPUT
        sprintf(fileName, filePattern, timeManager.getSteps());
        tracerManager.output(fileName);
#endif
    }
}
