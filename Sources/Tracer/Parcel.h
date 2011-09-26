#ifndef _Parcel_h_
#define _Parcel_h_

#include <string>
#include <vector>
#include "List.h"
#include "Polygon.h"

using std::string;
using std::vector;

class Parcel : public ListElement<Parcel>
{
public:
	Parcel();
	~Parcel();

    void reinit();

    void linkPolygon(Polygon *polygon);

    //! \brief Attach a tracer to the parcel.
    void attachTracer();

    //! \brief Update the attached tracers (e.g. calculate the density)
    void update();

private:
	Polygon *polygon;
    vector<double> mass;
    vector<double> density;
};

#endif