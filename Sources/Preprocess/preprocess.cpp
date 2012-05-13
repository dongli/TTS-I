#include "DelaunayDriver.hpp"
#include "PointManager.hpp"
#include "PolygonManager.hpp"
#include "ConfigTools.hpp"
#include "MeshManager.hpp"
#include "SCVT.hpp"
#include <netcdfcpp.h>

int main(int argc, char **argv)
{
    ConfigTools::parse(argv[1]);
    DelaunayDriver driver;
    // -------------------------------------------------------------------------
    if (ConfigTools::hasKey("use_points")) {
        // use specified points to generate polygons
        NOTICE("preprocess", "Reading in point data.")
        NcFile file(argv[1], NcFile::ReadOnly);
        if (!file.is_valid()) {
            cout << "[Error]: preprocess: Failed to open file " <<
            argv[1] << "." << endl;
            return 1;
        }
        int numPoint = file.get_dim("grid_size")->size();
        double lon[numPoint], lat[numPoint];
        file.get_var("grid_center_lon")->get(lon, numPoint);
        file.get_var("grid_center_lat")->get(lat, numPoint);
        file.close();
        NOTICE("preprocess", "Triangulating point data.")
        PointManager pointManager;
        pointManager.init(numPoint, lon, lat);
        driver.init(pointManager);
        driver.run();
        driver.calcCircumcenter();
    } else if (ConfigTools::hasKey("use_scvt")) {
        // use SCVT and a uniform density function to generate polygons
        int maxIteration, numGenerator;
        ConfigTools::read("max_iteration", maxIteration);
        ConfigTools::read("num_generator", numGenerator);
        int numLon = 360, numLat = 180;
        double dlon = PI2/numLon, dlat = PI/(numLat+1);
        double lon[numLon], lat[numLat];
        for (int i = 0; i < numLon; ++i)
            lon[i] = i*dlon;
        for (int j = 0; j < numLat; ++j)
            lat[j] = PI05-(j+1)*dlat;
        MeshManager meshManager;
        meshManager.init(numLon, numLat, lon, lat);
        const RLLMesh &mesh = meshManager.getMesh(PointCounter::Bound);
        SCVT::init(mesh.getNumLon(), mesh.getNumLat(),
                   mesh.lon.data(),  mesh.lat.data(), maxIteration);
        SCVT::getDensityFunction() = 1.0;
        SCVT::run(numGenerator, driver, "pp");
    }
    // -------------------------------------------------------------------------
    NOTICE("preprocess", "Organizing data for outputting.")
    PolygonManager polygonManager;
    polygonManager.init(driver);
    string fileName;
    ConfigTools::read("output_file", fileName);
    polygonManager.output(fileName);

    return 0;
}
