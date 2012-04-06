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
    char fileName[225], dirName[225], filePattern[50];
    clock_t start, end;
    start = clock();
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
#ifdef TTS_OUTPUT
    ConfigTools::read("output_file_pattern", filePattern);
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
    end = clock();
    cout << "[Timing]: tts_gamil_demo: ";
    cout << setprecision(5) << (double)(end-start)/CLOCKS_PER_SEC/60.0 << " minutes." << endl;
}
