#ifndef ProjectionTags_h
#define ProjectionTags_h

#include "Tags.hpp"

enum ProjectionTag {
    Approaching,
    Calculated,
    Crossing,
    CannotSplitPolygon
};

class ProjectionTags : public Tags<ProjectionTag, 4>
{
};

#endif
