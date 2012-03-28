#include "SCVT.h"
#include "DelaunayDriver.h"
#include "RandomNumber.h"
#include "Sphere.h"
#include <netcdfcpp.h>

#include "PolygonManager.h"

// density function
Array<double, 2> rho;
Array<double, 1> lonBnd, latBnd;

// running controls
int maxIteration;
double eps;

void SCVT::init(int numLon, int numLat, const double *lon, const double *lat)
{
    // -------------------------------------------------------------------------
    // Note: The input longitudes and latitudes are for the boundaries
    rho.resize(numLon-1, numLat-1);
    lonBnd.resize(numLon);
    latBnd.resize(numLat);
    for (int i = 0; i < numLon; ++i)
        lonBnd(i) = lon[i];
    for (int j = 0; j < numLat; ++j)
        latBnd(j) = lat[j];
    // -------------------------------------------------------------------------
    // Set running controls
    maxIteration = 500;
    eps = 1.0e-6;
}

/*
 * Function:
 *   getDensity
 * Purpose:
 *   Get the density value at given location (lon,lat) by using piecewise
 *   constant approximation.
 */
double getDensity(double lon, double lat)
{
    int i, j;
    bool isFound;
    // -------------------------------------------------------------------------
    isFound = false;
    for (i = 0; i < lonBnd.size()-1; ++i) {
        if (lon >= lonBnd(i) && lon < lonBnd(i+1)) {
            isFound = true;
            break;
        }
    }
    if (!isFound)
        i = 0;
    // -------------------------------------------------------------------------
    isFound = false;
    for (j = 0; j < latBnd.size()-1; ++j) {
        if (lat <= latBnd(j) && lat > latBnd(j+1)) {
            isFound = true;
            break;
        }
    }
    if (!isFound) {
        cout << "[Error]: SCVT::getDensity: Could not locate point!" << endl;
        cout << "[Debug]: SCVT::getDensity: Point coordinate is ";
        cout << setw(10) << setprecision(5) << lon*Rad2Deg;
        cout << setw(10) << setprecision(5) << lat*Rad2Deg;
        exit(-1);
    }
    return rho(i, j);
}

void SCVT::run(int numPoint, DelaunayDriver &driver)
{
    // -------------------------------------------------------------------------
    // 1. Initialize Voronoi generators using Monte Carlo method
#if defined (DEBUG) || defined (VERBOSE)
    cout << "[Notice]: SCVT::run: Initialize Voronoi generators." << endl;
#endif
    double lon[numPoint], lat[numPoint];
    double maxRho = max(rho);
    double ratio;
    RandomNumber::setRandomSeed();
    for (int i = 0; i < numPoint; ++i) {
        while (true) {
            lon[i] = RandomNumber::getRandomNumber(0.0, PI2);
            lat[i] = RandomNumber::getRandomNumber(-PI05, PI05);
            ratio = RandomNumber::getRandomNumber(0.0, 1.0);
            if (ratio <= getDensity(lon[i], lat[i])/maxRho)
                break;
        }
    }
#if defined (DEBUG) || defined (VERBOSE)
    cout << "[Notice]: SCVT::run: DONE!" << endl;
#endif
    // -------------------------------------------------------------------------
    // 2. Run the Lloyd's algorithm
#if defined (DEBUG) || defined (VERBOSE)
    cout << "[Notice]: SCVT::run: Running the Lloyd's algorithm." << endl;
#endif
    PointManager pointManager;
    pointManager.init(numPoint, lon, lat);
    driver.init(pointManager);
    bool isCriteriaMeet = false;
    for (int k = 0; k < maxIteration && !isCriteriaMeet; ++k) {
        double Linf = 0.0, L2 = 0.0;
        // =====================================================================
        // Run Delaunay triangulation and infer Voronoi diagram
        driver.reinit();
        driver.run();
        driver.calcCircumcenter();
#ifdef DEBUG
        {
            PolygonManager pm;
            char fileName[30];
            sprintf(fileName, "scvt_it%4.4d.nc", k);
            pm.init(driver);
            pm.output(fileName);
        }
#endif
        // =====================================================================
        // Calculate centroids of Voronoi cells (approximated)
        DelaunayVertex *DVT;
        // Evaluate the density on the generators (DVT)
        assert(numPoint == driver.DVT->size());
        double rhoDVT[numPoint];
        DVT = driver.DVT->front();
        for (int i = 0; i < driver.DVT->size(); ++i) {
            rhoDVT[i] = getDensity(DVT->point->getCoordinate().getLon(),
                                   DVT->point->getCoordinate().getLat());
            DVT = DVT->next;
        }
        DVT = driver.DVT->front();
        for (int i = 0; i < driver.DVT->size(); ++i) {
            Vector car = 0.0;
            double W = 0.0;
            const Coordinate &x0 = DVT->point->getCoordinate();
            DelaunayVertexPointer *linkDVT = DVT->topology.linkDVT.front();
            for (int j = 0; j < DVT->topology.linkDVT.size(); ++j) {
                int i1 = linkDVT->ptr->getID()-1;
                int i2 = linkDVT->next->ptr->getID()-1;
                const Coordinate &x1 = linkDVT->ptr->point->getCoordinate();
                const Coordinate &x2 = linkDVT->next->ptr->point->getCoordinate();
                double w0, w1, w2;
                w0 = x0.getX()*rhoDVT[i]+x1.getX()*rhoDVT[i1]+x2.getX()*rhoDVT[i2];
                w1 = x0.getY()*rhoDVT[i]+x1.getY()*rhoDVT[i1]+x2.getY()*rhoDVT[i2];
                w2 = x0.getZ()*rhoDVT[i]+x1.getZ()*rhoDVT[i1]+x2.getZ()*rhoDVT[i2];
                double area = Sphere::calcArea(x0, x1, x2);
                car(0) += area*w0;
                car(1) += area*w1;
                car(2) += area*w2;
                W += area*(rhoDVT[i]+rhoDVT[i1]+rhoDVT[i2]);
                linkDVT = linkDVT->next;
            }
            car /= W;
            car /= norm(car);
            Coordinate c0;
            c0.setCAR(car(0), car(1), car(2));
            double distance = Sphere::calcDistance(x0, c0)/Sphere::radius;
            L2 += distance*distance;
            if (distance > Linf) Linf = distance;
            lon[i] = c0.getLon();
            lat[i] = c0.getLat();
            DVT = DVT->next;
        }
        L2 = sqrt(L2/(1.0*numPoint));
        if (L2 < eps) {
            isCriteriaMeet = true;
        } else {
            DVT = driver.DVT->front();
            for (int i = 0; i < driver.DVT->size(); ++i) {
                DVT->point->setCoordinate(lon[i], lat[i]);
                DVT = DVT->next;
            }
        }
#if defined (DEBUG) || defined (VERBOSE)
        cout << "[Notice]: SCVT::run: Iteration " << k << ": ";
        cout << "L2 = " << L2 << " Linf = " << Linf << endl;
#endif
    }
#if defined (DEBUG) || defined (VERBOSE)
    cout << "[Notice]: SCVT::run: DONE!" << endl;
#endif
}

Array<double, 2> &SCVT::getDensityFunction()
{
    return rho;
}

void SCVT::outputDensityFunction(const char *fileName)
{
    NcFile file(fileName, NcFile::Replace);
    NcDim *lonDim = file.add_dim("lon", rho.extent(0));
    NcDim *latDim = file.add_dim("lat", rho.extent(1));
    NcDim *bndDim = file.add_dim("bnd", 2);
    NcVar *lonVar = file.add_var("lon", ncDouble, lonDim);
    NcVar *latVar = file.add_var("lat", ncDouble, latDim);
    NcVar *lonBndVar = file.add_var("lon_bnd", ncDouble, bndDim, lonDim);
    NcVar *latBndVar = file.add_var("lat_bnd", ncDouble, bndDim, latDim);
    NcVar *rhoVar = file.add_var("rho", ncDouble, latDim, lonDim);
    lonVar->add_att("long_name", "longitude");
    lonVar->add_att("units", "degree_east");
    latVar->add_att("long_name", "latitude");
    latVar->add_att("units", "degree_north");
    double lon[rho.extent(0)], lat[rho.extent(1)];
    double lonBnd_[2][rho.extent(0)], latBnd_[2][rho.extent(1)];
    for (int i = 0; i < rho.extent(0); ++i) {
        lon[i] = (lonBnd(i)+lonBnd(i+1))*0.5*Rad2Deg;
        lonBnd_[0][i] = lonBnd(i)*Rad2Deg;
        lonBnd_[1][i] = lonBnd(i+1)*Rad2Deg;
    }
    for (int j = 0; j < rho.extent(1); ++j) {
        lat[j] = (latBnd(j)+latBnd(j+1))*0.5*Rad2Deg;
        latBnd_[0][j] = latBnd(j)*Rad2Deg;
        latBnd_[1][j] = latBnd(j+1)*Rad2Deg;
    }
    double tmp[rho.extent(1)][rho.extent(0)];
    for (int i = 0; i < rho.extent(0); ++i)
        for (int j = 0; j < rho.extent(1); ++j)
            tmp[j][i] = rho(i, j);
    lonVar->put(lon, rho.extent(0));
    latVar->put(lat, rho.extent(1));
    lonBndVar->put(&lonBnd_[0][0], 2, rho.extent(0));
    latBndVar->put(&latBnd_[0][0], 2, rho.extent(1));
    rhoVar->put(&tmp[0][0], rho.extent(1), rho.extent(0));
    file.close();
}
