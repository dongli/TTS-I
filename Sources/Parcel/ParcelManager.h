#ifndef _ParcelManager_h_
#define _ParcelManager_h_

#include "PolygonManager.h"

class ParcelManager
{
public:
	ParcelManager();
	virtual ~ParcelManager();

    void construct(const string &fileName);

    void output(const string &fileName);

    PolygonManager polygonManager;
};

#endif
