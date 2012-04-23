#include "MovingVortices.h"
#include "TimeManager.h"
#include "Constants.h"
#include "ReportMacros.h"
#include "Sphere.h"

MovingVortices::MovingVortices()
{
    U0 = PI2*Sphere::radius/12.0/86400.0;
    alpha = PI05;
    rho0 = 3.0;
    axisPole.setSPH(PI, PI05-alpha);

    gamma = 5.0;
    xv0.setSPH(PI05, 0.0);
    Sphere::rotate(axisPole, xv0, xvr0);
}

MovingVortices::~MovingVortices()
{
}

void MovingVortices::calcVelocityField(FlowManager &flowManager)
{
    if (TimeManager::getSteps() != 0 && TimeManager::isFirstStep()) {
        // for restarting
        double time = TimeManager::getSeconds()-TimeManager::getTimeStep();
        calcVelocityField(flowManager, time);
    }
    calcVelocityField(flowManager, TimeManager::getSeconds());
}

void MovingVortices::calcVelocityField(FlowManager &flowManager, double time)
{
    Coordinate xvr;
    double lon = xvr0.getLon()+U0/Sphere::radius*time;
    if (lon > PI2) lon -= PI2;
    xvr.setSPH(lon, xvr0.getLat());
    Coordinate xv;
    Sphere::inverseRotate(axisPole, xv, xvr);

    const RLLMesh &umesh = flowManager.u.getMesh();
    const RLLMesh &vmesh = flowManager.v.getMesh();
    double u[umesh.getNumLon()-2][umesh.getNumLat()][1];
    double v[vmesh.getNumLon()-2][vmesh.getNumLat()][1];

    // -------------------------------------------------------------------------
    // normal regions
    for (int i = 1; i < umesh.getNumLon()-1; ++i)
        for (int j = 0; j < umesh.getNumLat(); ++j) {
            Coordinate x, xr;
            x.setSPH(umesh.lon(i), umesh.lat(j));
            Sphere::rotate(xv, x, xr);
            double ReOmega = Sphere::radius*omega(xr.getLat());
            double dlon = x.getLon()-xv.getLon();
            double tmp1, tmp2;
            tmp1 = cos(x.getLat())*cos(alpha);
            tmp2 = sin(x.getLat())*cos(x.getLon())*sin(alpha);
            double rotatePart = U0*(tmp1+tmp2);
            tmp1 = sin(xv.getLat())*cos(x.getLat());
            tmp2 = cos(xv.getLat())*cos(dlon)*sin(x.getLat());
            double deformPart = ReOmega*(tmp1-tmp2);
            u[i-1][j][0] = rotatePart+deformPart;
        }

    for (int i = 1; i < vmesh.getNumLon()-1; ++i)
        for (int j = 0; j < vmesh.getNumLat(); ++j) {
            Coordinate x, xr;
            x.setSPH(vmesh.lon(i), vmesh.lat(j));
            Sphere::rotate(xv, x, xr);
            double ReOmega = Sphere::radius*omega(xr.getLat());
            double dlon = x.getLon()-xv.getLon();
            double rotatePart = -U0*sin(x.getLon())*sin(alpha);
            double deformPart = ReOmega*cos(xv.getLat())*sin(dlon);
            v[i-1][j][0] = rotatePart+deformPart;
        }

    // -------------------------------------------------------------------------
    flowManager.update(&u[0][0][0], &v[0][0][0]);
}

inline double MovingVortices::rho(double lat) const
{
    return rho0*cos(lat);
}

double MovingVortices::omega(double latR) const
{
    static const double fac = 1.5*sqrt(3.0);
    double r = rho(latR); // radial distance of vortex
    double U; // Tangential velocity

    if (fabs(r) < 1.0e-12) {
        return 0.0;
    } else {
        U = U0/Sphere::radius*fac*tanh(r)/(cosh(r)*cosh(r));
        return U/r;
    }
}

#ifdef TTS_ONLINE
void MovingVortices::calcInitCond(MeshManager &meshManager,
                                  MeshAdaptor &meshAdaptor,
                                  TracerManager &tracerManager)
{
    // -------------------------------------------------------------------------
    // two dual meshes
    const RLLMesh &meshCnt = meshManager.getMesh(PointCounter::Center);
    const RLLMesh &meshBnd = meshManager.getMesh(PointCounter::Bound);
    // -------------------------------------------------------------------------
    // evaluate the initial condition on the RLL mesh of point counter
    tracerManager.registerTracer("test tracer 0", "test unit", meshManager);
    Field qt;
    qt.init(meshCnt, meshBnd);
    Array<double, 2> qtmp(meshCnt.getNumLon(), meshCnt.getNumLat());
#ifdef DEBUG
    double totalCellMass = 0.0;
#endif
    calcSolution(0.0, meshCnt.lon, meshCnt.lat, qtmp);
    for (int i = 0; i < meshCnt.getNumLon()-1; ++i)
        for (int j = 0; j < meshCnt.getNumLat(); ++j) {
            qt.values(i, j) = qtmp(i, j);
#ifdef DEBUG
            totalCellMass += qt.values(i, j).getNew()*meshBnd.area(i, j);
#endif
        }
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
    meshAdaptor.remap("test tracer 0", qt, tracerManager);
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
    meshAdaptor.remap("test tracer 0", tracerManager);
}
#endif

void MovingVortices::calcSolution(double time, const Array<double, 1> &lon,
                                  const Array<double, 1> &lat, Array<double, 2> &q)
{
    static double angleSpeed = U0/Sphere::radius;
    static double dlon = angleSpeed*TimeManager::getTimeStep();
    // -------------------------------------------------------------------------
    // calcuate the rotated coordinate of the north vortex center
    double lon_ = xvr0.getLon()+angleSpeed*time;
    if (lon_ > PI2) lon_ -= PI2;
    Coordinate xvr, xv;
    xvr.setSPH(lon_, xvr0.getLat());
    Sphere::inverseRotate(axisPole, xv, xvr);
    // -------------------------------------------------------------------------
    // check if reversal is necessary
    static bool doReverse = false;
    if (lon_ <= dlon || (lon_ > PI && lon_-PI <= dlon)) {
        NOTICE("MovingVortices::calcSolution", "Do reverse");
        doReverse = !doReverse;
    }
    // -------------------------------------------------------------------------
    Coordinate x, xr;
    for (int i = 0; i < q.extent(0); ++i)
        for (int j = 0; j < q.extent(1); ++j) {
            x.setSPH(lon(i), lat(j));
            Sphere::rotate(xv, x, xr);
            if (doReverse) {
                lon_ = xr.getLon()+PI;
                if (lon_ > PI2) lon_ -= PI2;
                xr.setSPH(lon_, xr.getLat());
            }
            q(i, j) = 1.0-tanh(rho(xr.getLat())/gamma*
                               sin(xr.getLon()-omega(xr.getLat())*time));
        }
}

void MovingVortices::calcSolution(Field &q)
{
    static double angleSpeed = U0/Sphere::radius;
    static double dlon = angleSpeed*TimeManager::getTimeStep();
    // -------------------------------------------------------------------------
    // calcuate the rotated coordinate of the north vortex center
    double lon = xvr0.getLon()+angleSpeed*TimeManager::getSeconds();
    if (lon > PI2) lon -= PI2;
    Coordinate xvr, xv;
    xvr.setSPH(lon, xvr0.getLat());
    Sphere::inverseRotate(axisPole, xv, xvr);
    // -------------------------------------------------------------------------
    // check if reversal is necessary
    static bool doReverse = false;
    if (lon <= dlon || (lon > PI && lon-PI <= dlon)) {
        NOTICE("MovingVortices::calcSolution", "Do reverse");
        doReverse = !doReverse;
    }
    // -------------------------------------------------------------------------
    const RLLMesh &mesh = q.getMesh(Field::Center);
    Coordinate x, xr;
    for (int i = 0; i < mesh.getNumLon()-1; ++i)
        for (int j = 0; j < mesh.getNumLat(); ++j) {
            x.setSPH(mesh.lon(i), mesh.lat(j));
            Sphere::rotate(xv, x, xr);
            if (doReverse) {
                lon = xr.getLon()+PI;
                if (lon > PI2) lon -= PI2;
                xr.setSPH(lon, xr.getLat());
            }
            q.values(i, j) = 1.0-tanh(rho(xr.getLat())/gamma*
                                      sin(xr.getLon()-omega(xr.getLat())*
                                          TimeManager::getSeconds()));
        }
}

void MovingVortices::calcSolution(MeshManager &meshManager,
                                  MeshAdaptor &meshAdaptor,
                                  TracerManager &tracerManager)
{
    // -------------------------------------------------------------------------
    // two dual meshes
    const RLLMesh &meshCnt = meshManager.getMesh(PointCounter::Center);
    const RLLMesh &meshBnd = meshManager.getMesh(PointCounter::Bound);
    // -------------------------------------------------------------------------
    // evaluate the initial condition on the RLL mesh of point counter
    Field q0;
    q0.init(meshCnt, meshBnd);
    calcSolution(q0);
    // -------------------------------------------------------------------------
    meshAdaptor.remap("test tracer", q0, tracerManager);
    // -------------------------------------------------------------------------
    meshAdaptor.remap("test tracer", tracerManager);
    
}
