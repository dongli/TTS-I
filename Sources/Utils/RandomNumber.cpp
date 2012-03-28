#include "RandomNumber.h"
#include <cstdlib>
#include <ctime>

void RandomNumber::setRandomSeed()
{
    srand(clock());
}

double RandomNumber::getRandomNumber(double a, double b) {
    const long max_rand = 1000000L;
    return a+(b-a)*(rand()%max_rand)/max_rand;
}

int RandomNumber::getRandomNumber(int a, int b)
{
    return rand()%(b-a)+a;
}
