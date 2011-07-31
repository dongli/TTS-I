#include "TimeManager.h"
#include "ReportMacros.h"
#include <iostream>
#include <iomanip>

using std::cout;
using std::endl;

bool TimeManager::isOnLine = false;
bool TimeManager::isClockSet = false;
Clock TimeManager::clock;
int TimeManager::startStep;
int TimeManager::endStep;

TimeManager::TimeManager()
{
#ifndef UNIT_TEST
    REPORT_ONLINE("TimeManager")
#endif
    isOnLine = true;
}

TimeManager::~TimeManager()
{
#ifndef UNIT_TEST
    REPORT_OFFLINE("TimeManager")
#endif
    isOnLine = true;
    isClockSet = false;
}

void TimeManager::setClock(Second dt, Second seconds, int steps)
{
    if (isClockSet)
        REPORT_ERROR("Clock has already been set.")
    clock.dt = dt;
    clock.seconds = seconds;
    startStep = steps;
    clock.steps = steps;
    isClockSet = true;
}

void TimeManager::setEndStep(int endStep)
{
    this->endStep = endStep;
}

void TimeManager::advance()
{
    clock.seconds += clock.dt;
    ++clock.steps;
    cout << "** step " << clock.steps << endl;
}
