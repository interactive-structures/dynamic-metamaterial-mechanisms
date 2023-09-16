#include "chipmunk/chipmunk.h"
#include <vector>
#include <unordered_map>
#include <memory>
#include "Position.hpp"

#pragma once

class SimulationBody {
    friend class SimulationSpace;
    //wrapper class for cpBody
    private:
        cpBody* myBody;
        cpSpace* mySpace;
        cpShape* myShape;
    public:
        SimulationBody() {
            myBody = nullptr;
            mySpace = nullptr;
            myShape = nullptr;
        }
        SimulationBody(cpSpace* space, cpBody* body, cpShape* shape) : mySpace(space), myBody(body), myShape(shape) {};
        Position getPos();
        Position getRot();
        Position getGlobalSegmentPosA();
        Position getGlobalSegmentPosB();
        Position getOffsetAlongSegment(double offset);
        void setPosition(Position pos);
        void changePosition(Position delta);
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
        void step() const;
        void setGravity(Position gravity) const;
        SimulationBody addSegmentBody(Position start, Position end, double mass, double radius) const;
        SimulationBody addCircleBody(Position center, double mass, double innerRadius, double outerRadius) const;
        SimulationBody addStaticSegmentBody(Position start, Position end, double radius) const;
        SimulationBody addKinematicBody(Position pos) const;
        SimulationConstraint pivotConstrain(SimulationBody bodyA, SimulationBody bodyB, Position anchorPos) const;
        SimulationConstraint rotarySpringConstrain(SimulationBody bodyA, SimulationBody bodyB, double restAngle, double stiffness, double damping) const;
        ~SimulationSpace();
};

typedef std::shared_ptr<SimulationSpace> SimulationSpacePtr;