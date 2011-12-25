#ifndef VertexTags_h
#define VertexTags_h

#include "Tags.h"

enum VertexTag {
    MayCrossEdge
};

class VertexTags : public Tags<VertexTag, 1>
{
};

#endif
