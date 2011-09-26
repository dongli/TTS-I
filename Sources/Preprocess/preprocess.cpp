#include "DelaunayDriver.h"
#include "PointManager.h"
#include "process_cocircularDT.h"
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
    driver.linkPoint(pointManager);
    driver.init();
    driver.calcircum();
    driver.output("delaunay");

    // -------------------------------------------------------------------------
    NOTICE("preprocess", "Processing cocircular Delaunay triangles.")
    process_cocircularDT(driver);

    // -------------------------------------------------------------------------
    NOTICE("preprocess", "Organizing data for outputting.")
    PolygonManager polygonManager;
    // translate DelaunayDriver data to PolygonManager data for unifying output
    polygonManager.polygons.create(numPoint);
    polygonManager.vertices.create(driver.DT->size());

    bool checked[numPoint];
    memset(checked, 0, numPoint*sizeof(bool));

    DelaunayVertex *DVT = driver.DVT->front();
    Polygon *polygon = polygonManager.polygons.front();
    for (int i = 0; i < driver.DVT->size(); ++i) {
        DelaunayVertexPointer *linkDVT = DVT->topology.linkDVT->front();
        DelaunayTrianglePointer *incidentDT = DVT->topology.incidentDT->front();
        for (int j = 0; j < DVT->topology.linkDVT->size(); ++j) {
            Edge *edge;
            if (checked[linkDVT->ptr->getID()-1]) {
                Polygon *p;
                p = polygonManager.polygons.at(linkDVT->ptr->getID()-1);
                EdgePointer *edgePointer = p->edgePointers.front();
                for (int k = 0; k < p->edgePointers.size(); ++k) {
                    if (edgePointer->edge->getEndPoint(FirstPoint)->getID() ==
                        incidentDT->ptr->getID() &&
                        edgePointer->edge->getEndPoint(SecondPoint)->getID() ==
                        incidentDT->prev->ptr->getID()) break;
                    edgePointer = edgePointer->next;
                }
                // use old edge
                edge = edgePointer->edge;
                edge->linkPolygon(OrientRight, polygon);
            } else {
                // set vertices
                Vertex *v1, *v2;
                Point *center;
                v1 = polygonManager.vertices.at(incidentDT->prev->ptr->getID()-1);
                v2 = polygonManager.vertices.at(incidentDT->ptr->getID()-1);
                center = &(incidentDT->prev->ptr->circumcenter);
                v1->setCoordinate(center->getCoordinate().getLon(),
                                  center->getCoordinate().getLat());
                center = &(incidentDT->ptr->circumcenter);
                v2->setCoordinate(center->getCoordinate().getLon(),
                                  center->getCoordinate().getLat());
                // create new edge
                polygonManager.edges.append(&edge);
                edge->linkEndPoint(FirstPoint, v1);
                edge->linkEndPoint(SecondPoint, v2);
                edge->linkPolygon(OrientLeft, polygon);
            }
            linkDVT = linkDVT->next;
            incidentDT = incidentDT->next;
        }
        checked[i] = true;
        DVT = DVT->next;
        polygon = polygon->next;
    }
    
    polygonManager.output(string(argv[2]));

    return 0;
}
