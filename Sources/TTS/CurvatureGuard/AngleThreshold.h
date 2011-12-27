#ifndef AngleThreshold_h
#define AngleThreshold_h

class Edge;

#include <string>
#include <blitz/array.h>

using std::string;
using blitz::Array;

namespace CurvatureGuard
{
    class AngleThreshold
    {
    public:
        AngleThreshold() {}
        ~AngleThreshold() {}

        static void init();

        static void calc(Edge *edge, double &a);
        static void calc(Edge *edge1, Edge *edge2, double &a);
        static void relax(Edge *edge1, Edge *edge2, double &a);

    private:
        static Array<double, 1> A, L, R;
    };
}

#endif
