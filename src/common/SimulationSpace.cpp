#include "SimulationSpace.hpp"

SimulationSpace::SimulationSpace(double timeStep) : timeStep(timeStep) {
    mySpace = cpSpaceNew();
}

void SimulationSpace::step() {
    cpSpaceStep(mySpace, timeStep);
}

void SimulationSpace::setGravity(Position gravity) {
    cpSpaceSetGravity(mySpace, cpv(gravity[0], gravity[1]));
}

SimulationBody SimulationSpace::addSegmentBody(Position start, Position end, double mass, double radius) {
    cpVect sv = cpv(start[0], start[1]), ev = cpv(end[0], end[1]);
    cpVect pos = (sv + ev) * (1.0 / 2.0);
    cpBody* body = cpBodyNew(mass, cpMomentForSegment(mass, sv, ev, radius));
    cpShape* shape = cpSegmentShapeNew(body, pos - sv, ev - pos, radius);
    cpSpaceAddBody(mySpace, body);
    cpSpaceAddShape(mySpace, shape);
    return SimulationBody(mySpace, body, shape);
}

SimulationBody SimulationSpace::addStaticSegmentBody(Position start, Position end, double radius) {
    cpVect sv = cpv(start[0], start[1]), ev = cpv(end[0], end[1]);
    cpVect pos = (sv + ev) * (1.0 / 2.0);
    cpBody* body = cpBodyNewStatic();
    cpShape* shape = cpSegmentShapeNew(body, pos - sv, ev - pos, radius);
    cpSpaceAddBody(mySpace, body);
    cpSpaceAddShape(mySpace, shape);
    return SimulationBody(mySpace, body, shape);
}

SimulationBody SimulationSpace::addKinematicBody(Position pos) {
    cpVect p = cpv(pos[0], pos[1]);
    cpBody* body = cpBodyNewKinematic();
    cpBodySetPosition(body, p);
    cpSpaceAddBody(mySpace, body);
    return SimulationBody(mySpace, body, nullptr);
}

void SimulationConstraint::setMaxForce(double constraintForce) {
    cpConstraintSetMaxForce(myConstraint, constraintForce);
}

double SimulationConstraint::getCurrentForce() {
    return cpConstraintGetImpulse(myConstraint) / timeStep;
}

double SimulationConstraint::getCurrentImpulse() {
    return cpConstraintGetImpulse(myConstraint);
}

Position SimulationBody::getPos() {
    cpVect pos = cpBodyGetPosition(myBody);
    return {pos.x, pos.y};
}

Position SimulationBody::getRot() {
    cpVect rot = cpBodyGetRotation(myBody);
    return {rot.x, rot.y};
}

SimulationConstraint SimulationSpace::pivotConstrain(SimulationBody bodyA, SimulationBody bodyB, Position anchorPos) {
    cpVect pos = cpv(anchorPos[0], anchorPos[1]);
    cpConstraint* cons = cpPivotJointNew(bodyA.myBody, bodyB.myBody, pos);
    cpSpaceAddConstraint(mySpace, cons);
    return SimulationConstraint(mySpace, cons, timeStep);
}

SimulationConstraint SimulationSpace::rotarySpringConstrain(SimulationBody bodyA, SimulationBody bodyB, double restAngle, double stiffness, double damping) {
    cpConstraint* cons = cpDampedRotarySpringNew(bodyA.myBody, bodyB.myBody, restAngle, stiffness,  damping);
    cpSpaceAddConstraint(mySpace, cons);
    return SimulationConstraint(mySpace, cons, timeStep);
}

SimulationSpace::~SimulationSpace() {
    cpSpaceFree(mySpace);
}