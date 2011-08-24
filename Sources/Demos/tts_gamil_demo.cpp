#include "Sphere.h"
#include "TimeManager.h"
#include "GAMILReader.h"
#include "TTS.h"

int main(int argc, char **argv)
{
    TimeManager timeManager;
    GAMILReader gamilReader;
    ParcelManager parcelManager;
    TTS tts;

    // -------------------------------------------------------------------------
    Sphere::setRadius(6371.299e3);
    tts.init();

    // -------------------------------------------------------------------------
    parcelManager.construct(argv[1]);

    // -------------------------------------------------------------------------
    gamilReader.construct("gamil_data", "tts.gamil.suv.*.nc");
    gamilReader.getVelocityField();
    //gamilReader.checkVelocityField();
    //gamilReader.flowManager.output("flow.nc");

    // -------------------------------------------------------------------------
    while (!timeManager.isFinished()) {
        tts.advect(gamilReader.meshManager,
                   gamilReader.flowManager, parcelManager);
        timeManager.advance();
        gamilReader.getVelocityField();
        char fileName[30];
        sprintf(fileName, "tts_test%5.5d.nc", timeManager.getSteps());
        parcelManager.output(fileName);
        //gamilReader.checkVelocityField();
        //gamilReader.flowManager.output("flow.nc");
    }
}
