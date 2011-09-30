#include "MeshAdaptor.h"
#include "PolygonManager.h"
#include "MeshManager.h"
#include "ReportMacros.h"
#include "Sphere.h"

MeshAdaptor::MeshAdaptor()
{
    REPORT_ONLINE("MeshAdaptor")
}

MeshAdaptor::~MeshAdaptor()
{
    REPORT_OFFLINE("MeshAdaptor")
}

void MeshAdaptor::init(const MeshManager &meshManager)
{
    const PointCounter &pointCounter = meshManager.pointCounter;

    overlapAreas.resize(pointCounter.points.shape());
    for (int i = 0; i < overlapAreas.extent(0); ++i)
        for (int j = 0; j < overlapAreas.extent(1); ++j)
            for (int k = 0; k < overlapAreas.extent(2); ++k)
                overlapAreas(i, j, k).resize(10);
}

void MeshAdaptor::remap(const PolygonManager &polygonManager,
                        const MeshManager &meshManager)
{
    //const RLLMesh &mesh = meshManager.getMesh(RLLMesh::BothHalf);
    //const PointCounter &pointCounter = meshManager.pointCounter;

    Polygon *polygon = polygonManager.polygons.front();
    for (int i = 0; i < polygonManager.polygons.size(); ++i) {
        int I1 = 9999, I2 = -I1, J1 = I1, J2 = -I1;
        EdgePointer *edgePointer = polygon->edgePointers.front();
        for (int j = 0; j < polygon->edgePointers.size(); ++j) {
            Vertex *vertex = edgePointer->getEndPoint(FirstPoint);
            int I = vertex->getLocation().i[4];
            int J = vertex->getLocation().j[4];
            I1 = fmin(I1, I);
            I2 = fmax(I2, I);
            J1 = fmin(J1, J);
            J2 = fmax(J2, J);
            edgePointer = edgePointer->next;
        }
        cout << I1 << "   " << I2 << endl;
        cout << J1 << "   " << J2 << endl;
        polygon = polygon->next;
    }
}