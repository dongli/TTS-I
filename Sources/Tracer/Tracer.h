#ifndef Tracer_h
#define Tracer_h

class Tracer
{
public:
    Tracer() {}
    virtual ~Tracer() {}

    void reinit() { mass = 0.0; }

    void setMass(double mass) { this->mass = mass; }
    void addMass(double mass) { this->mass += mass; }
    double getMass() const { return mass; }

    double mass;
    double density;
};

#endif
