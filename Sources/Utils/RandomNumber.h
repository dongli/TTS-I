#ifndef _RandomNumber_h_
#define _RandomNumber_h_

#include <cstdlib>
#include <ctime>

class RandomNumber
{
public:
    RandomNumber() {}
    virtual ~RandomNumber() {}

    static double getRandomNumber(double a, double b) {
        const long max_rand = 1000000L;
        srand(clock());
        return a+(b-a)*(rand()%max_rand)/max_rand;
    }

    static int getRandomNumber(int a, int b)
    {
        srand(clock());
        return rand()%(b-a)+a;
    }
};

#endif
