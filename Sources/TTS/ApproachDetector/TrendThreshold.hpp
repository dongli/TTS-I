#ifndef TrendThreshold_h
#define TrendThreshold_h

#include <blitz/array.h>

using blitz::Array;

namespace ApproachDetector
{
    class TrendThreshold
    {
    public:
        TrendThreshold() {}
        ~TrendThreshold() {}
        
        static void init();
        
        static void calc(double distance, double &percent);
        
    private:
        static Array<double, 1> P, D;
    };
}

#endif
