#ifndef CoverMask_h
#define CoverMask_h

#include <map>
#include <list>
#include <blitz/array.h>

using std::map;
using std::list;
using blitz::Array;

class CoverMask
{
public:
    CoverMask();
    ~CoverMask();
    
    void init(map<int, list<int> > &bndCellIdx, bool debug = false);
    void set(bool debug = false);

    // internal functions
    bool isCovered(int i0, int j0, int i, int j);
    void checkFalseCover(int i, int j);

    Array<int, 2> mask;
    Array<int, 1> idxI, idxJ;
};

#endif
