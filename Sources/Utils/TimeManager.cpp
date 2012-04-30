#include "TimeManager.hpp"
#include "ReportMacros.hpp"
#include <iostream>
#include <iomanip>

using std::cout;
using std::endl;

bool TimeManager::isOnLine = false;
bool TimeManager::isClockSet = false;
Clock TimeManager::clock;
int TimeManager::startStep;
int TimeManager::endStep;
vector<Alarm> TimeManager::alarms;

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

void TimeManager::setAlarm(const string &name, int freq)
{
    Alarm alarm;
    alarm.name = name;
    alarm.freq = freq;
    alarms.push_back(alarm);
}

bool TimeManager::isAlarmed(const string &name)
{
    for (int i = 0; i < alarms.size(); ++i)
        if (alarms[i].name == name &&
            getSteps()%alarms[i].freq == alarms[i].freq-1)
            return true;
    return false;
}

void TimeManager::setEndStep(int step)
{
    endStep = step;
}

void TimeManager::advance()
{
    clock.seconds += clock.dt;
    ++clock.steps;
    cout << "** step " << clock.steps << endl;
}
