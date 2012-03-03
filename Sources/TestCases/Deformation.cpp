#include "Deformation.h"
#include "Sphere.h"

Deformation::Deformation(SubCaseID subCaseID, InitCondID initCondID)
{
    this->subCaseID = subCaseID;
    this->initCondID = initCondID;
    T = 5.0;
}

Deformation::~Deformation()
{
}

void Deformation::calcVelocityField(FlowManager &flowManager)
{
    const RLLMesh &umesh = flowManager.u.getMesh();
    const RLLMesh &vmesh = flowManager.v.getMesh();
    double u[umesh.getNumLon()-2][umesh.getNumLat()][1];
    double v[vmesh.getNumLon()-1][vmesh.getNumLat()][1];
    //                          ^ Note the dimension size!
    double cosT = cos(PI*TimeManager::getSeconds()/T);
    double k;
    Coordinate x;

    // -------------------------------------------------------------------------
    // normal regions
    switch (subCaseID) {
        case Case1:
            k = 2.4;
            for (int i = 1; i < umesh.getNumLon()-1; ++i)
                for (int j = 0; j < umesh.getNumLat(); ++j) {
                    x.setSPH(umesh.lon(i), umesh.lat(j));
                    u[i-1][j][0] = k*pow(sin(x.getLon()*0.5), 2.0)*
                    sin(x.getLat()*2.0)*cosT;
                }
            for (int i = 0; i < vmesh.getNumLon()-1; ++i)
                for (int j = 0; j < vmesh.getNumLat(); ++j) {
                    x.setSPH(vmesh.lon(i), vmesh.lat(j));
                    v[i][j][0] = k*0.5*sin(x.getLon())*cos(x.getLat())*cosT;
                }
            break;
        case Case2:
            k = 2.0;
            for (int i = 1; i < umesh.getNumLon()-1; ++i)
                for (int j = 0; j < umesh.getNumLat(); ++j) {
                    x.setSPH(umesh.lon(i), umesh.lat(j));
                    u[i-1][j][0] = k*pow(sin(x.getLon()), 2.0)*
                    sin(x.getLat()*2.0)*cosT;
                }
            for (int i = 0; i < vmesh.getNumLon()-1; ++i)
                for (int j = 0; j < vmesh.getNumLat(); ++j) {
                    x.setSPH(vmesh.lon(i), vmesh.lat(j));
                    v[i][j][0] = k*sin(x.getLon()*2.0)*cos(x.getLat())*cosT;
                }
            break;
        case Case3:
            k = 1.0;
            for (int i = 1; i < umesh.getNumLon()-1; ++i)
                for (int j = 0; j < umesh.getNumLat(); ++j) {
                    x.setSPH(umesh.lon(i), umesh.lat(j));
                    u[i-1][j][0] = -k*pow(sin(x.getLon()*0.5), 2.0)*
                    sin(x.getLat()*2.0)*pow(cos(x.getLat()), 2.0)*cosT;
                }
            for (int i = 0; i < vmesh.getNumLon()-1; ++i)
                for (int j = 0; j < vmesh.getNumLat(); ++j) {
                    x.setSPH(vmesh.lon(i), vmesh.lat(j));
                    v[i][j][0] = k*0.5*sin(x.getLon())*
                    pow(cos(x.getLat()), 3.0)*cosT;
                }
            break;
        case Case4:
            k = 10.0*Sphere::radius/T;
            double c1 = PI2*TimeManager::getSeconds()/T;
            double c2 = PI2*Sphere::radius/T;
            for (int i = 1; i < umesh.getNumLon()-1; ++i)
                for (int j = 0; j < umesh.getNumLat(); ++j) {
                    x.setSPH(umesh.lon(i)-c1, umesh.lat(j));
                    u[i-1][j][0] = k*pow(sin(x.getLon()), 2.0)*
                    sin(x.getLat()*2.0)*cosT+c2*cos(x.getLat());
                }
            for (int i = 0; i < vmesh.getNumLon()-1; ++i)
                for (int j = 0; j < vmesh.getNumLat(); ++j) {
                    x.setSPH(vmesh.lon(i)-c1, vmesh.lat(j));
                    v[i][j][0] = k*sin(x.getLon()*2.0)*cos(x.getLat())*cosT;
                }
            break;
    }
    // -------------------------------------------------------------------------
    flowManager.update(&u[0][0][0], &v[0][0][0]);
}

#ifdef TTS_ONLINE
void Deformation::calcInitCond(MeshManager &meshManager,
                               MeshAdaptor &meshAdaptor,
                               TracerManager &tracerManager)
{
    // -------------------------------------------------------------------------
    // two dual meshes
    const RLLMesh &meshCnt = meshManager.getMesh(PointCounter::Center);
    const RLLMesh &meshBnd = meshManager.getMesh(PointCounter::Bound);
    // -------------------------------------------------------------------------
    // background air field with constant density
    tracerManager.registerTracer("air density", "test unit", meshManager);
    Field qa; qa.init(meshCnt, meshBnd);
    for (int i = 0; i < meshCnt.getNumLon()-1; ++i)
        for (int j = 0; j < meshCnt.getNumLat(); ++j) {
            qa.values(i, j, 0) = 1.0;
        }
    // -------------------------------------------------------------------------
    // evaluate the initial condition on the RLL mesh of point counter
    int numTracer;
    if (initCondID == CosineHills)
        numTracer = 2;
    else
        numTracer = 1;
    vector<string> tracerNames;
    tracerNames.resize(numTracer);
    vector<Field> qt;
    qt.resize(numTracer);
    for (int i = 0; i < numTracer; ++i) {
        char tracerName[30];
        sprintf(tracerName, "test tracer %d", i);
        tracerNames[i] = tracerName;
        tracerManager.registerTracer(tracerName, "test unit", meshManager);
        qt[i].init(meshCnt, meshBnd);
    }
    // -------------------------------------------------------------------------
#ifdef DEBUG
    double totalCellMass = 0.0;
#endif
    double hmax, b, c, r;
    Coordinate c0(5.0*PI/6.0, 0.0), c1(7.0*PI/6.0, 0.0);
    switch (initCondID) {
        case GaussianHills:
            hmax = 0.95; b = 5.0;
            for (int i = 0; i < meshCnt.getNumLon()-1; ++i)
                for (int j = 0; j < meshCnt.getNumLat(); ++j) {
                    Coordinate x(meshCnt.lon(i), meshCnt.lat(j));
                    Vector d0 = x.getCAR()-c0.getCAR();
                    Vector d1 = x.getCAR()-c1.getCAR();
                    qt[0].values(i, j) = hmax*(exp(-b*dot(d0, d0))+
                                              exp(-b*dot(d1, d1)));
#ifdef DEBUG
                    totalCellMass += qt[0].values(i, j).getNew()*
                                     meshBnd.area(i, j);
#endif
                }
            break;
        case CosineHills:
            hmax = 1.0; b = 0.1; c = 0.9; r = 0.5;
            for (int i = 0; i < meshCnt.getNumLon()-1; ++i)
                for (int j = 0; j < meshCnt.getNumLat(); ++j) {
                    Coordinate x(meshCnt.lon(i), meshCnt.lat(j));
                    double r0 = Sphere::calcDistance(x, c0);
                    double r1 = Sphere::calcDistance(x, c1);
                    if (r0 < r)
                        qt[0].values(i, j) = b+c*hmax*0.5*(1.0+cos(PI*r0/r));
                    else if (r1 < r)
                        qt[0].values(i, j) = b+c*hmax*0.5*(1.0+cos(PI*r1/r));
                    else
                        qt[0].values(i, j) = b;
#ifdef DEBUG
                    totalCellMass += qt[0].values(i, j).getNew()*
                                     meshBnd.area(i, j);
#endif
                }
            break;
        case SlottedCylinders:
            b = 0.1; c = 1.0; r = 0.5;
            for (int i = 0; i < meshCnt.getNumLon()-1; ++i)
                for (int j = 0; j < meshCnt.getNumLat(); ++j) {
                    Coordinate x(meshCnt.lon(i), meshCnt.lat(j));
                    double r0 = Sphere::calcDistance(x, c0);
                    double r1 = Sphere::calcDistance(x, c1);
                    if ((r0 <= r && fabs(x.getLon()-c0.getLon()) >= r/6.0) ||
                        (r1 <= r && fabs(x.getLon()-c1.getLon()) >= r/6.0))
                        qt[0].values(i, j) = c;
                    else if (r0 <= r && fabs(x.getLon()-c0.getLon()) < r/6.0 &&
                             x.getLat()-c0.getLat() < -5.0/12.0*r)
                        qt[0].values(i, j) = c;
                    else if (r1 <= r && fabs(x.getLon()-c1.getLon()) < r/6.0 &&
                             x.getLat()-c1.getLat() > 5.0/12.0*r)
                        qt[0].values(i, j) = c;
                    else
                        qt[0].values(i, j) = b;
#ifdef DEBUG
                    totalCellMass += qt[0].values(i, j).getNew()*
                                     meshBnd.area(i, j);
#endif
                }
            break;
    }
    // -------------------------------------------------------------------------
    if (numTracer == 2) {
        double a = -0.8, b = 0.9;
        for (int i = 0; i < meshCnt.getNumLon()-1; ++i)
            for (int j = 0; j < meshCnt.getNumLat(); ++j) {
                qt[1].values(i, j) = a*pow(qt[0].values(i, j).getNew(), 2.0)+b;
            }
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
    // remap the air density onto the polygons
    meshAdaptor.remap("air density", qa, tracerManager);
    // -------------------------------------------------------------------------
    // remap the initial condition onto the polygons
    for (int i = 0; i < numTracer; ++i)
        meshAdaptor.remap(tracerNames[i], qt[i], tracerManager);
    // -------------------------------------------------------------------------
#ifdef DEBUG
    double totalPolygonMass = 0.0;
    Polygon *polygon = tracerManager.polygonManager.polygons.front();
    for (int i = 0; i < tracerManager.polygonManager.polygons.size(); ++i) {
        totalPolygonMass += polygon->tracers[1].getMass();
        polygon = polygon->next;
    }
    cout << "Total cell mass is    " << setprecision(20) << totalCellMass << endl;
    cout << "Total polygon mass is " << setprecision(20) << totalPolygonMass << endl;
    cout << "Mass error is " << totalCellMass-totalPolygonMass << endl;
#endif
    // -------------------------------------------------------------------------
    // remap the air density onto the mesh that is used to analyze results
    meshAdaptor.remap("air density", tracerManager);
    // -------------------------------------------------------------------------
    // remap the initial condition onto the mesh that is used to analyze results
    for (int i = 0; i < numTracer; ++i)
        meshAdaptor.remap(tracerNames[i], tracerManager);
}
#endif
