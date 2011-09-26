#ifndef _Layers_h_
#define _Layers_h_

#include <vector>

using std::vector;

class Layers
{
public:
	enum LayerType {
		Full = 0, Half = 1
	};

    Layers();
	Layers(LayerType type, int numLev, double *lev);
	virtual ~Layers();

    void init(LayerType type, int numLev, double *lev);

    int getNumLev() const { return static_cast<int>(lev.size()); }

	LayerType type;
	vector<double> lev;
	vector<double> dlev;

    bool isConstructed;
};

#endif
