#include "ConfigTools.hpp"
#include "Sphere.hpp"
#include "TimeManager.hpp"
#include "GAMILReader.hpp"
#ifdef TTS_REZONE
#include "PolygonRezoner.hpp"
#endif
#include "TTS.hpp"
#include "CppHelper.hpp"

int main(int argc, char **argv)
{
    GAMILReader gamilReader;
    TracerManager tracerManager;
    TTS tts;
    clock_t start, end;
    start = clock();
    // -------------------------------------------------------------------------
    ConfigTools::parse(argv[1]);
    Sphere::setRadius(6371.299e3);
    // -------------------------------------------------------------------------
    tracerManager.init();
    tts.init();
    // -------------------------------------------------------------------------
    gamilReader.init();
    gamilReader.getTracerField(tracerManager);
#ifdef TTS_REZONE
    PolygonRezoner::rezone(gamilReader.meshManager, gamilReader.meshAdaptor,
                           gamilReader.flowManager, tracerManager);
#endif
    gamilReader.getVelocityField();
    gamilReader.checkVelocityField();
#ifdef TTS_OUTPUT
    char filePattern[50];
    ConfigTools::read("output_file_pattern", filePattern);
    tracerManager.output(to_string(TimeManager::getSteps(), filePattern));
#endif
    // -------------------------------------------------------------------------
    while (!TimeManager::isFinished()) {
        TimeManager::advance();
        gamilReader.getVelocityField();
        tts.advect(gamilReader.meshManager, gamilReader.meshAdaptor,
                   gamilReader.flowManager, tracerManager);
#ifdef TTS_OUTPUT
        tracerManager.output(to_string(TimeManager::getSteps(), filePattern));
#endif
    }
    end = clock();
    cout << "[Timing]: tts_gamil_demo: ";
    cout << setprecision(5) << (double)(end-start)/CLOCKS_PER_SEC/60.0 << " minutes." << endl;
}
