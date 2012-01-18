#include "CoverMask.h"
#include "ReportMacros.h"

CoverMask::CoverMask()
{
}

CoverMask::~CoverMask()
{
}

void CoverMask::init(map<int, list<int> > &bndCellIdx, bool debug)
{
    // -------------------------------------------------------------------------
    // get the bounding box
    int numCellLon = static_cast<int>(bndCellIdx.size()), numCellLat = 0;
    int J0 = -1, J1 = -1; // reference index
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
    // -------------------------------------------------------------------------
    // set the first guess cover mask
    mask.resize(numCellLon, numCellLat);
    mask = 0;
    idxI.resize(numCellLon);
    idxJ.resize(numCellLat);
    int i, j, J;
    // Note: The following complicated codes are due to the zonal periodic
    //       boundary condition, please forgive me!
    // second part if the polygon crosses lon = 0 meridinal line
    for (i = 0, itIdxIJ = itIdxIJ3; itIdxIJ != itIdxIJ4; ++itIdxIJ, ++i) {
        idxI(i) = (*itIdxIJ).first;
        j = (*itIdxIJ).second.front()-J0;
        list<int>::const_iterator itIdxJ1 = (*itIdxIJ).second.begin();
        idxJ(j) = *itIdxJ1;
        mask(i, j++) = 1;
        list<int>::const_iterator itIdxJ2 = itIdxJ1;
        for (++itIdxJ1; itIdxJ1 != (*itIdxIJ).second.end(); ++itIdxJ1) {
            if (*itIdxJ1-*itIdxJ2 != 1) {
                for (J = *itIdxJ2+1; J < *itIdxJ1; ++J) {
                    idxJ(j) = J;
                    mask(i, j++) = -1;
                }
                idxJ(j) = J;
                mask(i, j++) = 1;
            } else {
                idxJ(j) = *itIdxJ1;
                mask(i, j++) = 1;
            }
            itIdxJ2++;
        }
    }
    // first part
    for (itIdxIJ = itIdxIJ1; itIdxIJ != itIdxIJ2; ++itIdxIJ, ++i) {
        idxI(i) = (*itIdxIJ).first;
        j = (*itIdxIJ).second.front()-J0;
        list<int>::const_iterator itIdxJ1 = (*itIdxIJ).second.begin();
        idxJ(j) = *itIdxJ1;
        mask(i, j++) = 1;
        list<int>::const_iterator itIdxJ2 = itIdxJ1;
        for (++itIdxJ1; itIdxJ1 != (*itIdxIJ).second.end(); ++itIdxJ1) {
            if (*itIdxJ1-*itIdxJ2 != 1) {
                for (J = *itIdxJ2+1; J < *itIdxJ1; ++J) {
                    idxJ(j) = J;
                    mask(i, j++) = -1;
                }
                idxJ(j) = J;
                mask(i, j++) = 1;
            } else {
                idxJ(j) = *itIdxJ1;
                mask(i, j++) = 1;
            }
            itIdxJ2++;
        }
    }
#ifdef DEBUG
    if (debug) {
        for (i = 0; i < numCellLon; ++i)
            cout << setw(5) << idxI(i);
        cout << endl;
        for (j = 0; j < numCellLat; ++j)
            cout << idxJ(j) << endl;
        for (int jj = 0; jj < numCellLat; ++jj) {
            for (int ii = 0; ii < numCellLon; ++ii)
                cout << setw(3) << mask(ii, jj);
            cout << endl;
        }
    }
#endif
}

void CoverMask::set(bool debug)
{
    // set the cell that has been fully covered
    int i, j;
    for (i = 0; i < mask.extent(0); ++i)
        for (j = 0; j < mask.extent(1); ++j) {
            isCovered(-1, -1, i, j);
#ifdef DEBUG
            if (debug) {
                cout << i << ", " << j << ":" << endl;
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
#endif
        }
}

bool CoverMask::isCovered(int i0, int j0, int i, int j)
{
    // mask meanings:
    //  0 - cell is not covered by polygon
    //  1 - cell is partially covered by polygon
    //  2 - cell is fully covered by polygon
    // -1 - cell is potentially fully covered by polygon
    //  3 - cell as -1 cell but in checking
    // -------------------------------------------------------------------------
    if (mask(i, j) > 0)
        return true;
    else if (mask(i, j)== 0)
        return false;
    // -------------------------------------------------------------------------
    int ii, jj;
    // =========================================================================
    // check left-top cells
    for (ii = i-1, jj = j-1; ii >= 0 && jj >= 0; --ii, --jj)
        if (mask(ii, jj) > 0)
            break;
        else if (mask(ii, jj) == 0)
            goto return_not_covered;
        else if (mask(ii, jj) == -1) {
            if (ii == i0 && jj == j0)
                break;
            mask(i, j) = 3;
            if (!isCovered(i, j, ii, jj))
                goto return_not_covered;
            else
                break;
        }
    if (ii == -1 || jj == -1)
        goto return_not_covered;
    // =========================================================================
    // check left cells
    for (ii = i-1; ii >= 0; --ii)
        if (mask(ii, j) > 0)
            break;
        else if (mask(ii, j) == 0)
            goto return_not_covered;
        else if (mask(ii, jj) == -1) {
            if (ii == i0 && j == j0)
                break;
            mask(i, j) = 3;
            if (!isCovered(i, j, ii, j))
                goto return_not_covered;
            else
                break;
        }
    if (ii == -1)
        goto return_not_covered;
    // =========================================================================
    // check left-bottom cells
    for (ii = i-1, jj = j+1; ii >= 0 && jj < mask.extent(1); --ii, ++jj)
        if (mask(ii, jj) > 0)
            break;
        else if (mask(ii, jj) == 0)
            goto return_not_covered;
        else if (mask(ii, jj) == -1) {
            if (ii == i0 && jj == j0)
                break;
            mask(i, j) = 3;
            if (!isCovered(i, j, ii, jj))
                goto return_not_covered;
            else
                break;
        }
    if (ii == -1 || jj == mask.extent(1))
        goto return_not_covered;
    // =========================================================================
    // top cells
    for (jj = j-1; jj >= 0; --jj)
        if (mask(i, jj) > 0)
            break;
        else if (mask(i, jj) == 0)
            goto return_not_covered;
        else if (mask(i, jj) == -1) {
            if (i == i0 && jj == j0)
                break;
            mask(i, j) = 3;
            if (!isCovered(i, j, i, jj))
                goto return_not_covered;
            else
                break;
        }
    if (jj == -1)
        goto return_not_covered;
    // =========================================================================
    // check bottom cells
    for (jj = j+1; jj < mask.extent(1); ++jj)
        if (mask(i, jj) > 0)
            break;
        else if (mask(i, jj) == 0)
            goto return_not_covered;
        else if (mask(i, jj) == -1) {
            if (i == i0 && jj == j0)
                break;
            mask(i, j) = 3;
            if (!isCovered(i, j, i, jj))
                goto return_not_covered;
            else
                break;
        }
    if (jj == mask.extent(1))
        goto return_not_covered;
    // =========================================================================
    // check right-top cells
    for (ii = i+1, jj = j-1; ii < mask.extent(0) && jj >= 0; ++ii, --jj)
        if (mask(ii, jj) > 0)
            break;
        else if (mask(ii, jj) == 0)
            goto return_not_covered;
        else if (mask(ii, jj) == -1) {
            if (ii == i0 && jj == j0)
                break;
            mask(i, j) = 3;
            if (!isCovered(i, j, ii, jj))
                goto return_not_covered;
            else
                break;
        }
    if (ii == mask.extent(0) || jj == -1)
        goto return_not_covered;
    // =========================================================================
    // check right cells
    for (ii = i+1; ii < mask.extent(0); ++ii)
        if (mask(ii, j) > 0)
            break;
        else if (mask(ii, j) == 0)
            goto return_not_covered;
        else if (mask(ii, j) == -1) {
            if (ii == i0 && j == j0)
                break;
            mask(i, j) = 3;
            if (!isCovered(i, j, ii, j))
                goto return_not_covered;
            else
                break;
        }
    if (ii == mask.extent(0))
        goto return_not_covered;
    // =========================================================================
    // check right-bottom cells
    for (ii = i+1, jj = j+1;
         ii < mask.extent(0) && jj < mask.extent(1); ++ii, ++jj)
        if (mask(ii, jj) > 0)
            break;
        else if (mask(ii, jj) == 0)
            goto return_not_covered;
        else if (mask(ii, jj) == -1) {
            if (ii == i0 && jj == j0)
                break;
            mask(i, j) = 3;
            if (!isCovered(i, j, ii, jj))
                goto return_not_covered;
            else
                break;
        }
    if (ii == mask.extent(0) || jj == mask.extent(1))
        goto return_not_covered;
    // -------------------------------------------------------------------------
    mask(i, j) = 2;
    return true;
return_not_covered:
    // Note: The search route may forks, and some of the forked may return true,
    //       some may return false, and the order of the forked routes may cause
    //       different results, so we should check the result here to ensure
    //       there will be no false covered masks.
    mask(i, j) = 0;
    checkFalseCover(i, j);
    return false;
}

void CoverMask::checkFalseCover(int i, int j)
{
    assert(mask(i, j) == 0);
    // check the surrounding mask
    int ii, jj;
    // left-top cell
    if (i != 0 && j != 0) {
        ii = i-1; jj = j-1;
        if (mask(ii, jj) == 2) {
            mask(ii, jj) = 0;
            checkFalseCover(ii, jj);
        }
    }
    // left cell
    if (i != 0) {
        ii = i-1;
        if (mask(ii, j) == 2) {
            mask(ii, j) = 0;
            checkFalseCover(ii, j);
        }
    }
    // left-bottom cell
    if (i != 0 && j != mask.extent(1)) {
        ii = i-1; jj = j+1;
        if (mask(ii, jj) == 2) {
            mask(ii, jj) = 0;
            checkFalseCover(i, jj);
        }
    }
    // top cell
    if (j != 0) {
        jj = j-1;
        if (mask(i, jj) == 2) {
            mask(i, jj) = 0;
            checkFalseCover(i, jj);
        }
    }
    // bottom cell
    if (j != mask.extent(1)) {
        jj = j+1;
        if (mask(i, jj) == 2) {
            mask(i, jj) = 0;
            checkFalseCover(i, jj);
        }
    }
    // right-top cell
    if (i != mask.extent(0) && j != 0) {
        ii = i+1; jj = j-1;
        if (mask(ii, jj) == 2) {
            mask(ii, jj) = 0;
            checkFalseCover(ii, jj);
        }
    }
    // right cell
    if (i != mask.extent(0)) {
        ii = i+1;
        if (mask(ii, j) == 2) {
            mask(ii, j) = 0;
            checkFalseCover(ii, j);
        }
    }
    // right-bottom cell
    if (i != mask.extent(0) && j != mask.extent(1)) {
        ii = i+1; jj = j+1;
        if (mask(ii, jj) == 2) {
            mask(ii, jj) = 0;
            checkFalseCover(ii, jj);
        }
    }
}