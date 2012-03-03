#include "Point.h"
#include "ReportMacros.h"

Point::Point()
{
    x.init();
    isCoordinateSet = false;
}

Point::~Point()
{
}

void Point::reinit()
{
    isCoordinateSet = false;
}

void Point::setCoordinate(double lon, double lat, double lev, TimeLevel level)
{
    Coordinate x;
    x.setSPH(lon, lat, lev);
    if (level == NewTimeLevel) {
        if (isCoordinateSet)
            this->x.save();
        this->x.setNew(x);
        if (!isCoordinateSet) {
            this->x.save();
            isCoordinateSet = true;
        }
    } else if (level == OldTimeLevel) {
        this->x.setOld(x);
    }
}

void Point::setCoordinate(const Coordinate &x, TimeLevel level)
{
    if (level == NewTimeLevel) {
        if (isCoordinateSet)
            this->x.save();
        this->x.setNew(x);
        if (!isCoordinateSet) {
            this->x.save();
            isCoordinateSet = true;
        }
    } else if (level == OldTimeLevel) {
        this->x.setOld(x);
    }
}

const Coordinate &Point::getCoordinate(TimeLevel level) const
{
    if (level == NewTimeLevel) {
        return x.getNew();
    } else if (level == OldTimeLevel) {
        return x.getOld();
    } else {
        REPORT_ERROR("Unknown time level")
    }
}

void Point::setLocation(const Location &loc)
{
    this->loc.set(loc);
}

Point &Point::operator=(const Point &that)
{
    if (this != &that) {
        this->isCoordinateSet = that.isCoordinateSet;
        this->x = that.x;
        this->loc = that.loc;
    }
    return *this;
}

void Point::dump(int indentLevel) const
{
    for (int i = 0; i < indentLevel; ++i)
    cout << "  ";
    cout << "Old Coordinate: " << endl;
    x.getOld().dump(indentLevel+2);
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    cout << "New Coordinate: " << endl;
    x.getNew().dump(indentLevel+2);
}
