#include "Location.hpp"
#include <iostream>
#include <iomanip>

using std::setw;
using std::cout;
using std::endl;

Location::Location()
{
    i.resize(5);
    j.resize(5);
    for (int l = 0; l < i.size(); ++l) {
        i[l] = LOCATION_UNSET_INDEX;
        j[l] = LOCATION_UNSET_INDEX;
    }
	onPole = false;
	inPolarCap = false;
	pole = Null;
}

Location::~Location()
{
}

void Location::set(const Location &loc)
{
    for (int l = 0; l < this->i.size(); ++l) {
        this->i[l] = loc.i[l];
        this->j[l] = loc.j[l];
    }
    this->k = loc.k;
    this->onPole = loc.onPole;
    this->pole = loc.pole;
    this->inPolarCap = loc.inPolarCap;
}

void Location::dump() const
{
	cout << "Location:" << endl;
	cout << "  i = {";
    for (int l = 0; l < i.size()-1; ++l)
        cout << setw(8) << i[l] << ", ";
    cout << setw(8) << i.back() << "}" << endl;
	cout << "  j = {";
    for (int l = 0; l < i.size()-1; ++l)
        cout << setw(8) << j[l] << ", ";
    cout << setw(8) << j.back() << "}" << endl;
	cout << "  onPole     = " << onPole << endl;
	cout << "  pole       = ";
    if (pole == NorthPole) {
        cout << "North pole" << endl;
    } else if (pole == SouthPole) {
        cout << "South pole" << endl;
    }
	cout << "  inPolarCap = " << inPolarCap << endl;
}
