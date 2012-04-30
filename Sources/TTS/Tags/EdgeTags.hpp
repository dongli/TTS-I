#ifndef EdgeTags_h
#define EdgeTags_h

#include "Tags.hpp"

enum EdgeTag {
    SplitChecked
};

class EdgeTags : public Tags<EdgeTag, 1>
{
};

#endif
