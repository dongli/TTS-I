#ifndef Tracer_h
#define Tracer_h

class Tracer
{
public:
    Tracer() {}
    virtual ~Tracer() {}

    void reinit() {};

    double mass;
    double density;
};

#endif
