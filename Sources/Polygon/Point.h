#ifndef _Point_h_
#define _Point_h_

#include "Coordinate.h"
#include "Location.h"
#include "List.h"
#include "MultiTimeLevel.h"

class Point
{
public:
	Point();
	virtual ~Point();

    virtual void reinit();

	void setCoordinate(double lon, double lat, double lev = 0.0, TimeLevel level = NewTimeLevel);
    void setCoordinate(const Coordinate &, TimeLevel level = NewTimeLevel);
	const Coordinate &getCoordinate(TimeLevel level = NewTimeLevel) const;

	void setLocation(const Location &);
	const Location &getLocation() const { return loc; }

    Point &operator=(const Point &);

    void dump(int indentLevel = 0) const;

protected:
    bool isCoordinateSet;
	MultiTimeLevel<Coordinate, 2> x;
	Location loc;
};

#endif
