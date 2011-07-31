#include "ParcelManager.h"

ParcelManager::ParcelManager()
{
#ifndef UNIT_TEST
    REPORT_ONLINE("ParcelManager")
#endif
}

ParcelManager::~ParcelManager()
{
#ifndef UNIT_TEST
    REPORT_OFFLINE("ParcelManager")
#endif
}

void ParcelManager::construct(const string &fileName)
{
    polygonManager.construct(fileName);
}

void ParcelManager::output(const string &fileName)
{
    polygonManager.output(fileName);
}