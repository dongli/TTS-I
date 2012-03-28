#include "DelaunayDriver.h"
#include "PointManager.h"
#include "PolygonManager.h"
#include <netcdfcpp.h>
#include <string>
#include <iostream>

using namespace std;

typedef struct {
    int num;
    int *idx;
    int *ont;
} EdgeInfo;

int main(int argc, char **argv)
{
    // -------------------------------------------------------------------------
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
    
    // -------------------------------------------------------------------------
    NOTICE("preprocess", "Triangulating point data.")
    PointManager pointManager;
    DelaunayDriver driver;
    pointManager.init(numPoint, lon, lat);
    driver.init(pointManager);
    driver.run();
    driver.calcCircumcenter();
    driver.output("delaunay");

    // TODO: How to improve the quality of the Voronoi diagram?

    // -------------------------------------------------------------------------
    NOTICE("preprocess", "Organizing data for outputting.")
    PolygonManager polygonManager;
    polygonManager.init(driver);
    polygonManager.output(string(argv[2]));

    return 0;
}
