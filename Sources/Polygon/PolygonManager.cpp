#include "PolygonManager.h"
#include "Constants.h"
#include "TimeManager.h"
#include "DelaunayDriver.h"
#include "ReportMacros.h"
#include "SpecialPolygons.h"
#include <netcdfcpp.h>

PolygonManager::PolygonManager()
{
    vertices.setName("vertices");
    edges.setName("edges");
    polygons.setName("polygons");
#ifndef UNIT_TEST
    REPORT_ONLINE("PolygonManager")
#endif
}

PolygonManager::~PolygonManager()
{
    vertices.destroy();
    edges.destroy();
    polygons.destroy();
#ifndef UNIT_TEST
    REPORT_OFFLINE("PolygonManager")
#endif
}

void PolygonManager::reinit()
{
    vertices.recycle();
    edges.recycle();
    polygons.recycle();
}

void PolygonManager::init(const DelaunayDriver &driver)
{
    Vertex *vertex1, *vertex2;
    Edge *edge;
    EdgePointer *edgePointer;
    Polygon *polygon;
    // -------------------------------------------------------------------------
    // convert the Voronoi diagram into polygon representation
    polygons.create(driver.DVT->size());
    vertices.create(driver.DT->size());
    Polygon *polygonMap[polygons.size()];
    polygon = polygons.front();
    for (int i = 0; i < polygons.size(); ++i) {
        polygonMap[i] = polygon;
        polygon = polygon->next;
    }
    Vertex *vertexMap[vertices.size()];
    Vertex *vertex = vertices.front();
    for (int i = 0; i < vertices.size(); ++i) {
        vertexMap[i] = vertex;
        vertex = vertex->next;
    }
    bool checked[driver.DVT->size()];
    memset(checked, 0, polygons.size()*sizeof(bool));
    DelaunayVertex *DVT = driver.DVT->front();
    polygon = polygons.front();
    for (int i = 0; i < driver.DVT->size(); ++i) {
        DelaunayVertexPointer *linkDVT = DVT->topology.linkDVT->front();
        DelaunayTrianglePointer *incidentDT = DVT->topology.incidentDT->front();
        for (int j = 0; j < DVT->topology.linkDVT->size(); ++j) {
            if (checked[linkDVT->ptr->getID()-1]) {
                Polygon *p = polygonMap[linkDVT->ptr->getID()-1];
                edgePointer = p->edgePointers.front();
                int k;
                for (k = 0; k < p->edgePointers.size(); ++k) {
                    if (edgePointer->edge->getEndPoint(FirstPoint)->getID() ==
                        incidentDT->ptr->getID() &&
                        edgePointer->edge->getEndPoint(SecondPoint)->getID() ==
                        incidentDT->prev->ptr->getID()) break;
                    edgePointer = edgePointer->next;
                }
#ifdef DEBUG
                assert(k < p->edgePointers.size());
#endif
                // use old edge
                edgePointer->edge->linkPolygon(OrientRight, polygon);
            } else {
                // set vertices
                Point *center;
                vertex1 = vertexMap[incidentDT->prev->ptr->getID()-1];
                vertex2 = vertexMap[incidentDT->ptr->getID()-1];
                center = &(incidentDT->prev->ptr->circumcenter);
                vertex1->setCoordinate(center->getCoordinate().getLon(),
                                       center->getCoordinate().getLat());
                center = &(incidentDT->ptr->circumcenter);
                vertex2->setCoordinate(center->getCoordinate().getLon(),
                                       center->getCoordinate().getLat());
                // create new edge
                edges.append(&edge);
                edge->linkEndPoint(FirstPoint, vertex1);
                edge->linkEndPoint(SecondPoint, vertex2);
                edge->linkPolygon(OrientLeft, polygon);
                edge->calcNormVector();
                edge->calcLength();
            }
            linkDVT = linkDVT->next;
            incidentDT = incidentDT->next;
        }
        checked[i] = true;
        DVT = DVT->next;
        polygon->edgePointers.ring();
        polygon = polygon->next;
    }
    // -------------------------------------------------------------------------
    // remove short edges
    static const double minRatio = 0.05;
    polygons.startLoop(polygon);
    while (polygon != NULL) {
        // find out the maximum edge length and use it as a ruler
        double minLength = 1.0e34, maxLength = -1.0e34;
        edgePointer = polygon->edgePointers.front();
        for (int i = 0; i < polygon->edgePointers.size(); ++i) {
            if (minLength > edgePointer->edge->getLength())
                minLength = edgePointer->edge->getLength();
            if (maxLength < edgePointer->edge->getLength())
                maxLength = edgePointer->edge->getLength();
            edgePointer = edgePointer->next;
        }
        if (minLength/maxLength < minRatio) {
            polygon->dump("polygon");
            polygon->edgePointers.startLoop(edgePointer);
            do {
                if (edgePointer->edge->getLength()/maxLength < minRatio) {
                    // remove this edge
                    polygon->removeEdge(edgePointer, *this);
                }
                edgePointer = polygon->edgePointers.getNextElem();
            } while (!polygon->edgePointers.isLoopEnd());
        }
        polygon = polygon->next;
    }
    polygons.endLoop();
#ifdef TTS_OUTPUT
    // -------------------------------------------------------------------------
    // reindex the vertices and edges for outputting
    vertices.reindex();
    edges.reindex();
#endif
    // -------------------------------------------------------------------------
    polygons.startLoop(polygon);
    while (polygon != NULL) {
        if (polygon->edgePointers.size() == 2) {
            // TODO: Do we need to handle line polygons?
            SpecialPolygons::handleLinePolygon(*this, polygon);
        } else {
            EdgePointer *edgePointer = polygon->edgePointers.front();
            for (int j = 0; j < polygon->edgePointers.size(); ++j) {
                edgePointer->calcAngle();
                edgePointer = edgePointer->next;
            }
            polygon->calcArea();
        }
        polygon = polygons.getNextElem();
    }
    polygons.endLoop();
}

#ifdef TTS_ONLINE
void PolygonManager::init(const string &fileName)
{
    NOTICE("PolygonManager::init", "Reading polygons from \""+fileName+"\" ...");
    NcFile file(fileName.c_str(), NcFile::ReadOnly);
    if (!file.is_valid()) {
        REPORT_ERROR(string("Failed to open file "+fileName+"."))
    }

    NcError ncError(NcError::silent_nonfatal);

    if (TimeManager::onLine()) {
        NcAtt *timeAtt = file.get_att("time");
        NcAtt *timeStepAtt = file.get_att("time_step");
        NcAtt *stepsAtt = file.get_att("steps");
        if (timeAtt != NULL && timeStepAtt != NULL && stepsAtt != NULL) {
            TimeManager::reset();
            double dt = timeStepAtt->as_double(0);
            double second = timeAtt->as_double(0);
            double steps = stepsAtt->as_int(0);
            TimeManager::setClock(dt, second, steps);
        }
    }
    NcDim *numVertexDim = file.get_dim("num_total_vertex");
    if (numVertexDim == NULL) {
        Message message;
        message << "Failed to find \"num_total_vertex\" dimension in file \"";
        message << fileName << "\"!";
        REPORT_ERROR(message.str());
    }
    NcDim *numEdgeDim = file.get_dim("num_total_edge");
    if (numEdgeDim == NULL) {
        Message message;
        message << "Failed to find \"num_total_edge\" dimension in file \"";
        message << fileName << "\"!";
        REPORT_ERROR(message.str());
    }
    NcDim *numPolygonDim = file.get_dim("num_total_polygon");
    if (numPolygonDim == NULL) {
        Message message;
        message << "Failed to find \"num_total_polygon\" dimension in file \"";
        message << fileName << "\"!";
        REPORT_ERROR(message.str());
    }
    int numVertex = static_cast<int>(numVertexDim->size());
    int numEdge = static_cast<int>(numEdgeDim->size());
    int numPolygon = static_cast<int>(numPolygonDim->size());
    // -------------------------------------------------------------------------
    // vertices part
    vertices.create(numVertex);
    double *oldVtxLon = new double[numVertex];
    double *oldVtxLat = new double[numVertex];
    double *oldVtxLev = new double[numVertex];
    double *newVtxLon = new double[numVertex];
    double *newVtxLat = new double[numVertex];
    double *newVtxLev = new double[numVertex];
    file.get_var("old_vertex_lon")->get(oldVtxLon, numVertex);
    file.get_var("old_vertex_lat")->get(oldVtxLat, numVertex);
    file.get_var("new_vertex_lon")->get(newVtxLon, numVertex);
    file.get_var("new_vertex_lat")->get(newVtxLat, numVertex);
    if (file.get_var("old_vertex_lev") != NULL) {
        file.get_var("old_vertex_lev")->get(oldVtxLev, numVertex);
        file.get_var("new_vertex_lev")->get(newVtxLev, numVertex);
    } else {
        for (int i = 0; i < vertices.size(); ++i) {
            oldVtxLev[i] = 0.0;
            newVtxLev[i] = 0.0;
        }
    }
    // change the units from degree to rad
    for (int i = 0; i < numVertex; ++i) {
        oldVtxLon[i] /= Rad2Deg;
        oldVtxLat[i] /= Rad2Deg;
        newVtxLon[i] /= Rad2Deg;
        newVtxLat[i] /= Rad2Deg;
    }
    Vertex *vertexMap[vertices.size()];
    Vertex *vertex = vertices.front();
    for (int i = 0; i < vertices.size(); ++i) {
        vertexMap[i] = vertex;
        vertex->setCoordinate(newVtxLon[i], newVtxLat[i], newVtxLev[i], NewTimeLevel);
        vertex->setCoordinate(oldVtxLon[i], oldVtxLat[i], oldVtxLev[i], OldTimeLevel);
        vertex = vertex->next;
    }
    delete [] newVtxLon;
    delete [] newVtxLat;
    delete [] newVtxLev;
    delete [] oldVtxLon;
    delete [] oldVtxLat;
    delete [] oldVtxLev;
    // -------------------------------------------------------------------------
    // edges part
    edges.create(numEdge);
    int *firstPoint = new int[numEdge];
    int *secondPoint = new int[numEdge];
    file.get_var("first_point_idx")->get(firstPoint, numEdge);
    file.get_var("second_point_idx")->get(secondPoint, numEdge);
    Edge *edgeMap[edges.size()];
    Edge *edge = edges.front();
    for (int i = 0; i < edges.size(); ++i) {
        edgeMap[i] = edge;
        edge->linkEndPoint(FirstPoint, vertexMap[firstPoint[i]-1]);
        edge->linkEndPoint(SecondPoint, vertexMap[secondPoint[i]-1]);
        edge->calcNormVector();
        edge = edge->next;
    }
    delete [] firstPoint;
    delete [] secondPoint;
    // test point
    double *oldTestLon = new double[numEdge];
    double *oldTestLat = new double[numEdge];
    double *newTestLon = new double[numEdge];
    double *newTestLat = new double[numEdge];
    file.get_var("old_testpoint_lon")->get(oldTestLon, numEdge);
    file.get_var("old_testpoint_lat")->get(oldTestLat, numEdge);
    file.get_var("new_testpoint_lon")->get(newTestLon, numEdge);
    file.get_var("new_testpoint_lat")->get(newTestLat, numEdge);
    for (int i = 0; i < numEdge; ++i) {
        oldTestLon[i] /= Rad2Deg;
        oldTestLat[i] /= Rad2Deg;
        newTestLon[i] /= Rad2Deg;
        newTestLat[i] /= Rad2Deg;
    }
    edge = edges.front();
    for (int i = 0; i < numEdge; ++i) {
        Vertex *testPoint = edge->getTestPoint();
        testPoint->setCoordinate(oldTestLon[i], oldTestLat[i], OldTimeLevel);
        testPoint->setCoordinate(newTestLon[i], newTestLat[i], NewTimeLevel);
        edge = edge->next;
    }
    delete [] oldTestLon;
    delete [] oldTestLat;
    delete [] newTestLon;
    delete [] newTestLat;
    // -------------------------------------------------------------------------
    // polygons part
    polygons.create(numPolygon);
    int edgeNum[numPolygon];
    file.get_var("edge_num")->get(edgeNum, numPolygon);
    int numEdgeIdx = static_cast<int>(file.get_dim("num_edge_idx")->size());
    int *edgeIdx = new int[numEdgeIdx];
    int *edgeOnt = new int[numEdgeIdx];
    file.get_var("edge_idx")->get(edgeIdx, numEdgeIdx);
    file.get_var("edge_ont")->get(edgeOnt, numEdgeIdx);
    Polygon *polygon = polygons.front();
    int counter = 0;
    for (int i = 0; i < polygons.size(); ++i) {
        for (int j = 0; j < edgeNum[i]; ++j) {
            OrientStatus orient;
            if (edgeOnt[counter] == 0) {
                orient = OrientLeft;
            } else if (edgeOnt[counter] == 1) {
                orient = OrientRight;
            } else {
                string message = "Invalid edge_ont in file "+fileName+".";
                REPORT_ERROR(message.c_str());
            }
            edgeMap[edgeIdx[counter]-1]->linkPolygon(orient, polygon);
            counter++;
        }
        polygon->edgePointers.ring();
        // calculate the angles
        EdgePointer *edgePointer = polygon->edgePointers.front();
        for (int j = 0; j < polygon->edgePointers.size(); ++j) {
            edgePointer->calcAngle();
            edgePointer = edgePointer->next;
        }
        polygon->calcArea();
        polygon = polygon->next;
    }
    delete [] edgeIdx;
    delete [] edgeOnt;
    // -------------------------------------------------------------------------
    file.close();
}
#endif

void PolygonManager::output(const string &fileName)
{
    // -------------------------------------------------------------------------
    NcFile file(fileName.c_str(), NcFile::Replace);
    if (!file.is_valid()) {
        string message = "Failed to open file "+fileName+".";
        REPORT_ERROR(message.c_str());
    }
    // -------------------------------------------------------------------------
    // time information
    if (TimeManager::onLine()) {
        file.add_att("time", TimeManager::getSeconds());
        file.add_att("time_step", TimeManager::getTimeStep());
        file.add_att("steps", TimeManager::getSteps());
    }
    // -------------------------------------------------------------------------
    // dimensions
    NcDim *numVertexDim = file.add_dim("num_total_vertex", vertices.size());
    NcDim *numEdgeDim = file.add_dim("num_total_edge", edges.size());
    NcDim *numPolygonDim = file.add_dim("num_total_polygon", polygons.size());
    // -------------------------------------------------------------------------
    // vertices part
    NcVar *oldVtxLonVar = file.add_var("old_vertex_lon", ncDouble, numVertexDim);
    NcVar *oldVtxLatVar = file.add_var("old_vertex_lat", ncDouble, numVertexDim);
    NcVar *newVtxLonVar = file.add_var("new_vertex_lon", ncDouble, numVertexDim);
    NcVar *newVtxLatVar = file.add_var("new_vertex_lat", ncDouble, numVertexDim);
    oldVtxLonVar->add_att("long_name", "old vertex longitude");
    oldVtxLonVar->add_att("units", "degree_east");
    oldVtxLatVar->add_att("long_name", "old vertex latitude");
    oldVtxLatVar->add_att("units", "degree_north");
    newVtxLonVar->add_att("long_name", "new vertex longitude");
    newVtxLonVar->add_att("units", "degree_east");
    newVtxLatVar->add_att("long_name", "new vertex latitude");
    newVtxLatVar->add_att("units", "degree_north");
    double *oldVtxLon = new double[vertices.size()];
    double *oldVtxLat = new double[vertices.size()];
    double *newVtxLon = new double[vertices.size()];
    double *newVtxLat = new double[vertices.size()];
    Vertex *vertex = vertices.front();
    for (int i = 0; i < vertices.size(); ++i) {
        oldVtxLon[i] = vertex->getCoordinate(OldTimeLevel).getLon()*Rad2Deg;
        oldVtxLat[i] = vertex->getCoordinate(OldTimeLevel).getLat()*Rad2Deg;
        newVtxLon[i] = vertex->getCoordinate(NewTimeLevel).getLon()*Rad2Deg;
        newVtxLat[i] = vertex->getCoordinate(NewTimeLevel).getLat()*Rad2Deg;
        vertex = vertex->next;
    }
    oldVtxLonVar->put(oldVtxLon, vertices.size());
    oldVtxLatVar->put(oldVtxLat, vertices.size());
    newVtxLonVar->put(newVtxLon, vertices.size());
    newVtxLatVar->put(newVtxLat, vertices.size());
    delete [] oldVtxLon;
    delete [] oldVtxLat;
    delete [] newVtxLon;
    delete [] newVtxLat;
    // -------------------------------------------------------------------------
    // edges part
    NcVar *firstPointVar = file.add_var("first_point_idx", ncInt, numEdgeDim);
    NcVar *secondPointVar = file.add_var("second_point_idx", ncInt, numEdgeDim);
    firstPointVar->add_att("long_name", "first point index of edge");
    secondPointVar->add_att("long_name", "second point index of edge");
    int *idx1 = new int[edges.size()];
    int *idx2 = new int[edges.size()];
    Edge *edge = edges.front();
    for (int i = 0; i < edges.size(); ++i) {
        idx1[i] = edge->getEndPoint(FirstPoint)->getID();
        idx2[i] = edge->getEndPoint(SecondPoint)->getID();
        edge = edge->next;
    }
    firstPointVar->put(idx1, edges.size());
    secondPointVar->put(idx2, edges.size());
    delete [] idx1;
    delete [] idx2;
    // test point
    NcVar *oldTestLonVar = file.add_var("old_testpoint_lon", ncDouble, numEdgeDim);
    NcVar *oldTestLatVar = file.add_var("old_testpoint_lat", ncDouble, numEdgeDim);
    NcVar *newTestLonVar = file.add_var("new_testpoint_lon", ncDouble, numEdgeDim);
    NcVar *newTestLatVar = file.add_var("new_testpoint_lat", ncDouble, numEdgeDim);
    oldTestLonVar->add_att("long_name", "old test point longitude");
    oldTestLonVar->add_att("units", "degree_east");
    oldTestLatVar->add_att("long_name", "old test point latitude");
    oldTestLatVar->add_att("units", "degree_north");
    newTestLonVar->add_att("long_name", "new test point longitude");
    newTestLonVar->add_att("units", "degree_east");
    newTestLatVar->add_att("long_name", "new test point latitude");
    newTestLatVar->add_att("units", "degree_north");
#if defined TTS_ONLINE || PREPROCESS
    double *oldTestLon = new double[edges.size()];
    double *oldTestLat = new double[edges.size()];
    double *newTestLon = new double[edges.size()];
    double *newTestLat = new double[edges.size()];
    edge = edges.front();
    for (int i = 0; i < edges.size(); ++i) {
        Vertex *testPoint = edge->getTestPoint();
        oldTestLon[i] = testPoint->getCoordinate(OldTimeLevel).getLon()*Rad2Deg;
        oldTestLat[i] = testPoint->getCoordinate(OldTimeLevel).getLat()*Rad2Deg;
        newTestLon[i] = testPoint->getCoordinate(NewTimeLevel).getLon()*Rad2Deg;
        newTestLat[i] = testPoint->getCoordinate(NewTimeLevel).getLat()*Rad2Deg;
        edge = edge->next;
    }
    oldTestLonVar->put(oldTestLon, edges.size());
    oldTestLatVar->put(oldTestLat, edges.size());
    newTestLonVar->put(newTestLon, edges.size());
    newTestLatVar->put(newTestLat, edges.size());
    delete [] oldTestLon;
    delete [] oldTestLat;
    delete [] newTestLon;
    delete [] newTestLat;
#endif
    // -------------------------------------------------------------------------
    // polygons part
    NcVar *edgeNumVar = file.add_var("edge_num", ncInt, numPolygonDim);
    NcVar *areaVar = file.add_var("area", ncDouble, numPolygonDim);
    edgeNumVar->add_att("long_name", "edge number of polygon");
    areaVar->add_att("long_name", "polygon area");
    int edgeNum[polygons.size()];
    double area[polygons.size()];
    int counter = 0;
    Polygon *polygon = polygons.front();
    for (int i = 0; i < polygons.size(); ++i) {
        edgeNum[i] = polygon->edgePointers.size();
        area[i] = polygon->getArea();
        counter += edgeNum[i];
        polygon = polygon->next;
    }
    int numEdgeIdx = counter;
    NcDim *numEdgeIdxDim = file.add_dim("num_edge_idx", numEdgeIdx);
    NcVar *edgeIdxVar = file.add_var("edge_idx", ncInt, numEdgeIdxDim);
    NcVar *edgeOntVar = file.add_var("edge_ont", ncInt, numEdgeIdxDim);
    edgeIdxVar->add_att("long_name", "edge index of polygon");
    edgeOntVar->add_att("long_name", "edge orientation of polygon");
    int *edgeIdx = new int[counter];
    int *edgeOnt = new int[counter];
    counter = 0;
    polygon = polygons.front();
    for (int i = 0; i < polygons.size(); ++i) {
        EdgePointer *edgePointer = polygon->edgePointers.front();
        for (int j = 0; j < polygon->edgePointers.size(); ++j) {
            edgeIdx[counter] = edgePointer->edge->getID();
            edgeOnt[counter] = edgePointer->orient;
            counter++;
            edgePointer = edgePointer->next;
        }
        polygon = polygon->next;
    }
    edgeNumVar->put(edgeNum, polygons.size());
    areaVar->put(area, polygons.size());
    edgeIdxVar->put(edgeIdx, numEdgeIdx);
    edgeOntVar->put(edgeOnt, numEdgeIdx);
    delete [] edgeIdx;
    delete [] edgeOnt;
    // -------------------------------------------------------------------------
    file.close();
}
