#include "SolidRotation.h"
#include "TimeManager.h"
#include "Constants.h"
#include "ReportMacros.h"
#include "Sphere.h"

SolidRotation::SolidRotation()
{
    angleSpeed = PI2/12.0/86400.0;
    U0 = Sphere::radius*angleSpeed;
    alpha = PI05;
    axisPole.setSPH(PI, PI05-alpha);
    CR0.setSPH(PI05, PI05);
    R = Sphere::radius/3.0;
    H0 = 1000.0;
}

SolidRotation::~SolidRotation()
{
}

void SolidRotation::calcVelocityField(FlowManager &flowManager)
{
    const RLLMesh &umesh = flowManager.u.getMesh();
    const RLLMesh &vmesh = flowManager.v.getMesh();
    double u[umesh.getNumLon()-2][umesh.getNumLat()][1];
    double v[vmesh.getNumLon()-1][vmesh.getNumLat()][1];
    //                          ^ Note the dimension size!
    // -------------------------------------------------------------------------
    // normal regions
    for (int i = 1; i < umesh.getNumLon()-1; ++i)
        for (int j = 0; j < umesh.getNumLat(); ++j) {
            Coordinate x;
            x.setSPH(umesh.lon(i), umesh.lat(j));
            double tmp1, tmp2;
            tmp1 = cos(x.getLat())*cos(alpha);
            tmp2 = sin(x.getLat())*cos(x.getLon())*sin(alpha);
            double rotatePart = U0*(tmp1+tmp2);
            u[i-1][j][0] = rotatePart;
            //  ^ Note the index!
        }
    for (int i = 0; i < vmesh.getNumLon()-1; ++i)
        for (int j = 0; j < vmesh.getNumLat(); ++j) {
            Coordinate x;
            x.setSPH(vmesh.lon(i), vmesh.lat(j));
            double rotatePart = -U0*sin(x.getLon())*sin(alpha);
            v[i][j][0] = rotatePart;
        }
    // -------------------------------------------------------------------------
    flowManager.update(&u[0][0][0], &v[0][0][0]);
}

void SolidRotation::calcInitCond(MeshManager &meshManager,
                                 MeshAdaptor &meshAdaptor,
                                 TracerManager &tracerManager)
{
    // -------------------------------------------------------------------------
    // two dual meshes
    const RLLMesh &meshCnt = meshManager.getMesh(PointCounter::Center);
    const RLLMesh &meshBnd = meshManager.getMesh(PointCounter::Bound);
    // -------------------------------------------------------------------------
    // evaluate the initial condition on the RLL mesh of point counter
    tracerManager.registerTracer("test tracer", "test unit", meshManager);
    Field qt;
    qt.init(meshCnt, meshBnd);
    calcSolution(qt);
#ifdef DEBUG
    double totalCellMass = 0.0;
    for (int i = 0; i < meshCnt.getNumLon(); ++i)
        for (int j = 0; j < meshCnt.getNumLat(); ++j)
            totalCellMass += qt.values(i, j).getNew()*meshBnd.area(i, j);
#endif
    // -------------------------------------------------------------------------
    // check the location polygon vertices
    Vertex *vertex = tracerManager.polygonManager.vertices.front();
    for (int i = 0; i < tracerManager.polygonManager.vertices.size(); ++i) {
        Location loc;
        meshManager.checkLocation(vertex->getCoordinate(), loc, vertex);
        vertex->setLocation(loc);
        vertex = vertex->next;
    }
    // -------------------------------------------------------------------------
    meshAdaptor.adapt(tracerManager, meshManager);
    // -------------------------------------------------------------------------
    meshAdaptor.remap("test tracer", qt, tracerManager);
    // -------------------------------------------------------------------------
#ifdef DEBUG
    double totalPolygonMass = 0.0;
    Polygon *polygon = tracerManager.polygonManager.polygons.front();
    for (int i = 0; i < tracerManager.polygonManager.polygons.size(); ++i) {
        totalPolygonMass += polygon->tracers[0].getMass();
        polygon = polygon->next;
    }
    cout << "Total cell mass is    " << setprecision(20) << totalCellMass << endl;
    cout << "Total polygon mass is " << setprecision(20) << totalPolygonMass << endl;
    cout << "Mass error is " << totalCellMass-totalPolygonMass << endl;
#endif
    // -------------------------------------------------------------------------
    meshAdaptor.remap("test tracer", tracerManager);
}

void SolidRotation::calcSolution(Field &q)
{
    double lon = CR0.getLon()+angleSpeed*TimeManager::getSeconds();
    CR0.setSPH(lon, CR0.getLat());
    Sphere::inverseRotate(axisPole, C0, CR0);

    const RLLMesh &mesh = q.getMesh(Field::Center);
    for (int i = 0; i < mesh.getNumLon(); ++i)
        for (int j = 0; j < mesh.getNumLat(); ++j) {
            Coordinate x(mesh.lon(i), mesh.lat(j));
            double d = Sphere::calcDistance(C0, x);
            if (d < R)
                q.values(i, j) = H0*(1.0+cos(PI*d/R))/2.0;
            else
                q.values(i, j) = 0.0;
        }
}
