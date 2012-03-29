#include "CoverMask.h"
#include "Polygon.h"
#include "RLLMesh.h"
#include "Sphere.h"
#include "ReportMacros.h"

CoverMask::CoverMask(const RLLMesh &mesh)
{
    this->mesh = &mesh;
}

CoverMask::~CoverMask()
{
}

void CoverMask::init(Polygon *polygon, map<int, list<int> > &bndCellIdx,
                     Location::Pole pole, const RLLMesh &meshBnd, bool debug)
{
    isCycle = pole != Location::Null;
    // -------------------------------------------------------------------------
    // get the bounding box
    int numCellLon = static_cast<int>(bndCellIdx.size()), numCellLat = 0;
    int J0 = -1, J1 = -1; // reference index
    int offset = 0; // offset due to the inclusion of north pole
    map<int, list<int> >::iterator itIdxIJ1 = bndCellIdx.begin();
    map<int, list<int> >::iterator itIdxIJ2 = bndCellIdx.end();
    map<int, list<int> >::iterator itIdxIJ3 = bndCellIdx.end();
    map<int, list<int> >::iterator itIdxIJ4 = bndCellIdx.end();
    map<int, list<int> >::iterator itIdxIJ  = bndCellIdx.begin();
    map<int, list<int> >::iterator itIdxIJ0 = bndCellIdx.begin();
    for (++itIdxIJ0; itIdxIJ != bndCellIdx.end(); ++itIdxIJ, ++itIdxIJ0) {
        if (itIdxIJ0 != bndCellIdx.end() &&
            (*itIdxIJ).first+1 != (*itIdxIJ0).first) {
            itIdxIJ2 = itIdxIJ0;
            itIdxIJ3 = itIdxIJ0;
        }
        (*itIdxIJ).second.sort();
        (*itIdxIJ).second.unique();
        if (J0 == -1)
            J0 = (*itIdxIJ).second.front();
        else if (J0 > (*itIdxIJ).second.front())
            J0 = (*itIdxIJ).second.front();
        if (J1 == -1)
            J1 = (*itIdxIJ).second.back();
        else if (J1 < (*itIdxIJ).second.back())
            J1 = (*itIdxIJ).second.back();
    }
    numCellLat = J1-J0+1;
    if (pole == Location::NorthPole) {
        offset = J0;
        numCellLat += J0;
    } else if (pole == Location::SouthPole) {
        offset = numCellLat;
        numCellLat += meshBnd.getNumLat()-2-J1;
    }
    // -------------------------------------------------------------------------
    // initialize the index map
    idxI.resize(numCellLon);
    idxJ.resize(numCellLat);
    if (pole == Location::NorthPole)
        for (int j = 0; j < J0; ++j)
            idxJ(j) = j;
    else if (pole == Location::SouthPole) {
        int i = 1;
        for (int j = offset; j < numCellLat; ++j)
            idxJ(j) = J1+(i++);
        offset = 0;
    }
    // -------------------------------------------------------------------------
    // set the first guess cover mask
    mask.resize(numCellLon, numCellLat);
    mask = NoOverlap;
    int i = 0, j, J;
    // =========================================================================
    // Note: The following complicated codes are due to the zonal periodic
    //       boundary condition, please forgive me!
    // second part if the polygon crosses lon = 0 meridinal line
    for (itIdxIJ = itIdxIJ3; itIdxIJ != itIdxIJ4; ++itIdxIJ, ++i) {
        idxI(i) = (*itIdxIJ).first;
        j = (*itIdxIJ).second.front()-J0+offset;
        list<int>::const_iterator itIdxJ1 = (*itIdxIJ).second.begin();
        idxJ(j) = *itIdxJ1;
        mask(i, j++) = CrossedByEdges;
        list<int>::const_iterator itIdxJ2 = itIdxJ1;
        for (++itIdxJ1; itIdxJ1 != (*itIdxIJ).second.end(); ++itIdxJ1) {
            if (*itIdxJ1-*itIdxJ2 != 1) {
                for (J = *itIdxJ2+1; J < *itIdxJ1; ++J) {
                    idxJ(j) = J;
                    mask(i, j++) = PotentialCovered;
                }
                idxJ(j) = J;
                mask(i, j++) = CrossedByEdges;
            } else {
                idxJ(j) = *itIdxJ1;
                mask(i, j++) = CrossedByEdges;
            }
            itIdxJ2++;
        }
    }
    // first part
    for (itIdxIJ = itIdxIJ1; itIdxIJ != itIdxIJ2; ++itIdxIJ, ++i) {
        idxI(i) = (*itIdxIJ).first;
        j = (*itIdxIJ).second.front()-J0+offset;
        list<int>::const_iterator itIdxJ1 = (*itIdxIJ).second.begin();
        idxJ(j) = *itIdxJ1;
        mask(i, j++) = CrossedByEdges;
        list<int>::const_iterator itIdxJ2 = itIdxJ1;
        for (++itIdxJ1; itIdxJ1 != (*itIdxIJ).second.end(); ++itIdxJ1) {
            if (*itIdxJ1-*itIdxJ2 != 1) {
                for (J = *itIdxJ2+1; J < *itIdxJ1; ++J) {
                    idxJ(j) = J;
                    mask(i, j++) = PotentialCovered;
                }
                idxJ(j) = J;
                mask(i, j++) = CrossedByEdges;
            } else {
                idxJ(j) = *itIdxJ1;
                mask(i, j++) = CrossedByEdges;
            }
            itIdxJ2++;
        }
    }
    // =========================================================================
    // set the cells that will be determined to be covered due the coverage of
    // pole by the polygon
    // Note: use "ray casting algorithm" to test if pole is covered
    if (pole != Location::Null) {
        bool isPoleCovered = false;
        EdgePointer *edgePointer = polygon->edgePointers.front();
        for (int i = 0; i < polygon->edgePointers.size(); ++i) {
            Vertex *vertex1 = edgePointer->getEndPoint(FirstPoint);
            Vertex *vertex2 = edgePointer->getEndPoint(SecondPoint);
            double x1 = vertex1->getCoordinate().getX();
            double y1 = vertex1->getCoordinate().getY();
            double x2 = vertex2->getCoordinate().getX();
            double y2 = vertex2->getCoordinate().getY();
            if (((y1 > 0.0) != (y2 > 0.0)) &&
                ((x2-x1)*(-y1)/(y2-y1)+x1 > 0.0))
                isPoleCovered = !isPoleCovered;
            edgePointer = edgePointer->next;
        }
        if (isPoleCovered) {
            if (pole == Location::NorthPole) {
                for (int i = 0; i < mask.extent(0); ++i)
                    for (int j = 0; j < mask.extent(1); ++j)
                        if (mask(i, j) == NoOverlap)
                            mask(i, j) = FullyCoveredNearPole;
                        else
                            break;
            } else {
                for (int i = 0; i < mask.extent(0); ++i)
                    for (int j = mask.extent(1)-1; j >= 0; --j)
                        if (mask(i, j) == NoOverlap)
                            mask(i, j) = FullyCoveredNearPole;
                        else
                            break;
            }
        }
    }
return_label:
#ifdef DEBUG
    if (debug) dump();
#endif
    return;
}

void CoverMask::searchCover(Polygon *polygon, bool debug)
{
    // set the cell that has been fully covered
    bool isFirstCoveredCell;
    for (int i = 0; i < mask.extent(0); ++i)
        for (int j = 0; j < mask.extent(1); ++j) {
            isFirstCoveredCell = true;
            isCovered(polygon, -1, -1, i, j, isFirstCoveredCell);
#ifdef DEBUG
            if (debug) dump(i, j);
#endif
        }
}

bool CoverMask::isCovered(Polygon *polygon, int i0, int j0, int i, int j,
                          bool &isFirstCoveredCell)
{
    // -------------------------------------------------------------------------
    if (mask(i, j) > 0)
        return true;
    else if (mask(i, j) == 0)
        return false;
    // -------------------------------------------------------------------------
    int ii, jj;
    // =========================================================================
    // check left-top cells
    ii = i-1, jj = j-1;
    while (true) {
        if (jj == -1)
            goto return_not_covered;
        if (ii < 0)
            if (isCycle)
                ii += mask.extent(0);
            else
                goto return_not_covered;
        if (mask(ii, jj) > 0)
            break;
        else if (mask(ii, jj) == NoOverlap)
            goto return_not_covered;
        else if (mask(ii, jj) == PotentialCovered) {
            if (ii == i0 && jj == j0)
                break;
            mask(i, j) = PotentialCoveredPassed;
            if (!isCovered(polygon, i, j, ii, jj, isFirstCoveredCell))
                goto return_not_covered;
            else
                break;
        }
        --ii, --jj;
    }
    // =========================================================================
    // left cells
    ii = i-1;
    while (true) {
        if (ii < 0)
            if (isCycle)
                ii += mask.extent(0);
            else
                goto return_not_covered;
        if (mask(ii, j) > 0)
            break;
        else if (mask(ii, j) == NoOverlap)
            goto return_not_covered;
        else if (mask(ii, j) == PotentialCovered) {
            if (ii == i0 && j == j0)
                break;
            mask(i, j) = PotentialCoveredPassed;
            if (!isCovered(polygon, i, j, ii, j, isFirstCoveredCell))
                goto return_not_covered;
            else
                break;
        }
        --ii;
    }
    // =========================================================================
    // left-right cells
    ii = i-1, jj = j+1;
    while (true) {
        if (jj == mask.extent(1))
            goto return_not_covered;
        if (ii < 0)
            if (isCycle)
                ii += mask.extent(0);
            else
                goto return_not_covered;
        if (mask(ii, jj) > 0)
            break;
        else if (mask(ii, jj) == NoOverlap)
            goto return_not_covered;
        else if (mask(ii, jj) == PotentialCovered) {
            if (ii == i0 && jj == j0)
                break;
            mask(i, j) = PotentialCoveredPassed;
            if (!isCovered(polygon, i, j, ii, jj, isFirstCoveredCell))
                goto return_not_covered;
            else
                break;
        }
        --ii, ++jj;
    }
    // =========================================================================
    // top cells
    jj = j-1;
    while (true) {
        if (jj == -1)
            goto return_not_covered;
        if (mask(i, jj) > 0)
            break;
        else if (mask(i, jj) == PotentialCovered) {
            if (i == i0 && jj == j0)
                break;
            mask(i, j) = PotentialCoveredPassed;
            if (!isCovered(polygon, i, j, i, jj, isFirstCoveredCell))
                goto return_not_covered;
            else
                break;
        }
        --jj;
    }
    // =========================================================================
    // bottom cells
    jj = j+1;
    while (true) {
        if (jj == mask.extent(1))
            goto return_not_covered;
        if (mask(i, jj) > 0)
            break;
        else if (mask(i, jj) == PotentialCovered) {
            if (i == i0 && jj == j0)
                break;
            mask(i, j) = PotentialCoveredPassed;
            if (!isCovered(polygon, i, j, i, jj, isFirstCoveredCell))
                goto return_not_covered;
            else
                break;
        }
        ++jj;
    }
    // =========================================================================
    // right-top cells
    ii = i+1, jj = j-1;
    while (true) {
        if (jj == -1)
            goto return_not_covered;
        if (ii == mask.extent(0))
            if (isCycle)
                ii = 0;
            else
                goto return_not_covered;
        if (mask(ii, jj) > 0)
            break;
        else if (mask(ii, jj) == PotentialCovered) {
            if (ii == i0 && jj == j0)
                break;
            mask(i, j) = PotentialCoveredPassed;
            if (!isCovered(polygon, i, j, ii, jj, isFirstCoveredCell))
                goto return_not_covered;
            else
                break;
        }
        ++ii, --jj;
    }
    // =========================================================================
    // right cells
    ii = i+1;
    while (true) {
        if (ii == mask.extent(0))
            if (isCycle)
                ii = 0;
            else
                goto return_not_covered;
        if (mask(ii, j) > 0)
            break;
        else if (mask(ii, j) == PotentialCovered) {
            if (ii == i0 && j == j0)
                break;
            mask(i, j) = PotentialCoveredPassed;
            if (!isCovered(polygon, i, j, ii, j, isFirstCoveredCell))
                goto return_not_covered;
            else
                break;
        }
        ++ii;
    }
    // =========================================================================
    // right-bottom cells
    ii = i+1, jj = j+1;
    while (true) {
        if (jj == mask.extent(1))
            goto return_not_covered;
        if (ii == mask.extent(0))
            if (isCycle)
                ii = 0;
            else
                goto return_not_covered;
        if (mask(ii, jj) > 0)
            break;
        else if (mask(ii, jj) == PotentialCovered) {
            if (ii == i0 && jj == j0)
                break;
            mask(i, j) = PotentialCoveredPassed;
            if (!isCovered(polygon, i, j, ii, jj, isFirstCoveredCell))
                goto return_not_covered;
            else
                break;
        }
        ++ii, ++jj;
    }
    // -------------------------------------------------------------------------
    if (isFirstCoveredCell) {
        // use "ray casting algorithm" to ensure the cell is truely covered
        Coordinate X0(mesh->lon(idxI(i)+1), mesh->lat(idxJ(j)));
        bool isCovered = false;
        EdgePointer *edgePointer = polygon->edgePointers.front();
        for (int k = 0; k < polygon->edgePointers.size(); ++k) {
            Vertex *vertex1 = edgePointer->getEndPoint(FirstPoint);
            Vertex *vertex2 = edgePointer->getEndPoint(SecondPoint);
            const Coordinate X1 = vertex1->getCoordinate();
            const Coordinate X2 = vertex2->getCoordinate();
            Coordinate X1r, X2r;
            Sphere::rotate(X0, X1, X1r);
            Sphere::rotate(X0, X2, X2r);
            double x1 = X1r.getX();
            double y1 = X1r.getY();
            double x2 = X2r.getX();
            double y2 = X2r.getY();
            if (((y1 > 0.0) != (y2 > 0.0)) &&
                ((x2-x1)*(-y1)/(y2-y1)+x1 > 0.0))
                isCovered = !isCovered;
            edgePointer = edgePointer->next;
        }
        isFirstCoveredCell = false;
        if (isCovered) {
            mask(i, j) = FullyCovered;
            return true;
        } else {
            mask(i, j) = NoOverlap;
            return false;
        }
    } else {
        mask(i, j) = FullyCovered;
        return true;
    }
return_not_covered:
    // Note: The search route may forks, and some of the forked may return true,
    //       some may return false, and the order of the forked routes may cause
    //       different results, so we should check the result here to ensure
    //       there will be no false covered masks.
    mask(i, j) = NoOverlap;
    checkFalseCover(i, j);
    return false;
}

void CoverMask::checkFalseCover(int i, int j)
{
    assert(mask(i, j) == NoOverlap);
    // check the surrounding mask
    int ii, jj;
    // left-top cell
    if (i != 0 && j != 0) {
        ii = i-1; jj = j-1;
        if (mask(ii, jj) == FullyCovered) {
            mask(ii, jj) = NoOverlap;
            checkFalseCover(ii, jj);
        }
    }
    // left cell
    if (i != 0) {
        ii = i-1;
        if (mask(ii, j) == FullyCovered) {
            mask(ii, j) = NoOverlap;
            checkFalseCover(ii, j);
        }
    }
    // left-bottom cell
    if (i != 0 && j != mask.extent(1)-1) {
        ii = i-1; jj = j+1;
        if (mask(ii, jj) == FullyCovered) {
            mask(ii, jj) = NoOverlap;
            checkFalseCover(i, jj);
        }
    }
    // top cell
    if (j != 0) {
        jj = j-1;
        if (mask(i, jj) == FullyCovered) {
            mask(i, jj) = NoOverlap;
            checkFalseCover(i, jj);
        }
    }
    // bottom cell
    if (j != mask.extent(1)-1) {
        jj = j+1;
        if (mask(i, jj) == FullyCovered) {
            mask(i, jj) = NoOverlap;
            checkFalseCover(i, jj);
        }
    }
    // right-top cell
    if (i != mask.extent(0)-1 && j != 0) {
        ii = i+1; jj = j-1;
        if (mask(ii, jj) == FullyCovered) {
            mask(ii, jj) = NoOverlap;
            checkFalseCover(ii, jj);
        }
    }
    // right cell
    if (i != mask.extent(0)-1) {
        ii = i+1;
        if (mask(ii, j) == FullyCovered) {
            mask(ii, j) = NoOverlap;
            checkFalseCover(ii, j);
        }
    }
    // right-bottom cell
    if (i != mask.extent(0)-1 && j != mask.extent(1)-1) {
        ii = i+1; jj = j+1;
        if (mask(ii, jj) == FullyCovered) {
            mask(ii, jj) = NoOverlap;
            checkFalseCover(ii, jj);
        }
    }
}

void CoverMask::dump(int i, int j)
{
    cout << "---------------------------------------------------------" << endl;
    for (int jj = 0; jj < mask.extent(1); ++jj) {
        for (int ii = 0; ii < mask.extent(0); ++ii) {
            if (ii == i && jj == j)
                cout << " *" << setw(1) << mask(ii, jj);
            else
                cout << setw(3) << mask(ii, jj);
        }
        cout << endl;
    }
}
