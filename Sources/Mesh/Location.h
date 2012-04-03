#ifndef Location_h
#define Location_h

#include <vector>

#define LOCATION_UNSET_INDEX -999

class Location
{
public:
    enum Pole {
        NorthPole = 0, SouthPole = 1, Null = 2
    };

	Location();
	virtual ~Location();

    void set(const Location &loc);

	void dump() const;

	std::vector<int> i, j;
    int k;
	Pole pole;
	bool onPole;
	bool inPolarCap;
};

#endif
