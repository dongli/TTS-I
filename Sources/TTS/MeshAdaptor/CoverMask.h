#ifndef CoverMask_h
#define CoverMask_h

#include <map>
#include <list>
#include <blitz/array.h>
#include "Location.h"

class Polygon;
class RLLMesh;

using std::map;
using std::list;
using blitz::Array;

class CoverMask
{
public:
    CoverMask();
    ~CoverMask();

    enum MaskType {
        NoOverlap = 0, CrossedByEdges = 1,
        PotentialCovered = -1, PotentialCoveredPassed = 4,
        FullyCovered = 2, FullyCoveredNearPole = 3 
    };    

    void init(Polygon *polygon, map<int, list<int> > &bndCellIdx,
              Location::Pole pole, const RLLMesh &meshBnd, bool debug = false);
    void setMask(int i, int j, MaskType);
    void searchCover(bool debug = false);

    // internal functions
    bool isCovered(int i0, int j0, int i, int j);
    void checkFalseCover(int i, int j);
    void dump(int i = -1, int j = -1);

    Array<int, 2> mask;
    Array<int, 1> idxI, idxJ;
    bool isCycle;
};

#endif
