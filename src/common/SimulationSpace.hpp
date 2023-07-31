#include "chipmunk/chipmunk.h"
#include <vector>
#include <unordered_map>

using std::vector;

typedef vector<double> Position;


class SimulationBody {
    friend class SimulationSpace;
    //wrapper class for cpBody
    private:
        cpBody* myBody;
        cpSpace* mySpace;
        cpShape* myShape;
    public:
        SimulationBody(cpSpace* space, cpBody* body, cpShape* shape) : mySpace(space), myBody(body), myShape(shape) {};
        Position getPos();
        Position getRot();
};

class SimulationConstraint {
    friend class SimulationSpace;
    //wrapper class for cpConstraint
    private:
        cpConstraint* myConstraint;
        cpSpace* mySpace;
        double timeStep;
    public:
        SimulationConstraint(cpSpace* space, cpConstraint* constraint, double timestep) : mySpace(space), myConstraint(constraint), timeStep(timestep) {};
        void setMaxForce(double maxForce);
        double getCurrentForce();
        double getCurrentImpulse();
};

class SimulationSpace {
    //wrapper class for cpSpace
    private:
        double timeStep;
        cpSpace *mySpace;
    public:
        SimulationSpace(double timestep);
        void step();
        void setGravity(Position gravity);
        SimulationBody addSegmentBody(Position start, Position end, double mass, double radius);
        SimulationBody addStaticSegmentBody(Position start, Position end, double radius);
        SimulationBody addKinematicBody(Position pos);
        SimulationConstraint pivotConstrain(SimulationBody bodyA, SimulationBody bodyB, Position anchorPos);
        SimulationConstraint rotarySpringConstrain(SimulationBody bodyA, SimulationBody bodyB, double restAngle, double stiffness, double damping);
        ~SimulationSpace();
};