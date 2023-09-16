#include "SimulationSpace.hpp"
#include <iostream>


SimulationSpace::SimulationSpace(double timeStep) : timeStep(timeStep) {
    mySpace = cpSpaceNew();
}

void SimulationSpace::step() const {
    cpSpaceStep(mySpace, timeStep);
}

void SimulationSpace::setGravity(Position gravity) const {
    cpSpaceSetGravity(mySpace, cpv(gravity[0], gravity[1]));
}

SimulationBody SimulationSpace::addSegmentBody(Position start, Position end, double mass, double radius) const {
    cpVect sv = cpv(start[0], start[1]), ev = cpv(end[0], end[1]);
    cpVect pos = (sv + ev) * (1.0 / 2.0);
    std::cout << "MY POSITION IS " << pos.x << " " << pos.y << std::endl;
    std::cout << "BUT A IS " << sv.x << " " << sv.y << "AND B IS" << ev.x << " " << ev.y << std::endl;
    cpBody* body = cpBodyNew(mass, cpMomentForSegment(mass, sv, ev, radius));
    cpShape* shape = cpSegmentShapeNew(body, sv - pos, ev - pos, radius);
    cpBodySetPosition(body, pos);
    cpSpaceAddBody(mySpace, body);
    cpSpaceAddShape(mySpace, shape);
    return SimulationBody(mySpace, body, shape);
}

SimulationBody SimulationSpace::addCircleBody(Position center, double mass, double innerRadius, double outerRadius) const {
    cpVect pos = cpv(center[0], center[1]);
    std::cout << "MY POSITION IS " << pos.x << " " << pos.y << std::endl;
    cpBody* body = cpBodyNew(mass, cpMomentForCircle(mass, innerRadius, outerRadius, cpvzero));
    cpShape* shape = cpCircleShapeNew(body, outerRadius, cpvzero);
    cpBodySetPosition(body, pos);
    cpSpaceAddBody(mySpace, body);
    cpSpaceAddShape(mySpace, shape);
    return SimulationBody(mySpace, body, shape);
}

SimulationBody SimulationSpace::addStaticSegmentBody(Position start, Position end, double radius) const {
    cpVect sv = cpv(start[0], start[1]), ev = cpv(end[0], end[1]);
    cpBody* body = cpSpaceGetStaticBody(mySpace);
    cpShape* shape = cpSegmentShapeNew(body, sv, ev, radius);
    cpSpaceAddShape(mySpace, shape);
    return SimulationBody(mySpace, body, shape);
}

SimulationBody SimulationSpace::addKinematicBody(Position pos) const {
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

cpVect getWorldPosA(cpBody* body, cpShape *segment) {
    cpVect rotated = cpBodyGetRotation(body);
    cpVect translatedPosA = cpvrotate(cpSegmentShapeGetA(segment), rotated);
    return cpBodyGetPosition(body) + translatedPosA;
}

cpVect getWorldPosB(cpBody* body, cpShape *segment) {
    cpVect rotated = cpBodyGetRotation(body);
    cpVect translatedPosB = cpvrotate(cpSegmentShapeGetB(segment), rotated);
    return cpBodyGetPosition(body) + translatedPosB;
}


Position SimulationBody::getGlobalSegmentPosA() {
    cpVect pos = getWorldPosA(myBody, myShape);
    return {pos.x, pos.y};
}

Position SimulationBody::getOffsetAlongSegment(double offset)
{
    cpVect rotated = cpBodyGetRotation(myBody);
    cpVect b = cpSegmentShapeGetB(myShape);
    cpVect offsetVec = cpvmult(b, offset / cpvlength(b));
    cpVect translatedPosB = cpvrotate(offsetVec, rotated);
    cpVect pos = cpBodyGetPosition(myBody) + translatedPosB;
    return {pos.x, pos.y};
}

Position SimulationBody::getGlobalSegmentPosB() {
    cpVect pos = getWorldPosB(myBody, myShape);
    return {pos.x, pos.y};
}

void SimulationBody::setPosition(Position pos) {
    cpBodySetPosition(myBody, cpv(pos[0], pos[1]));
}

void SimulationBody::changePosition(Position delta) {
    cpBodySetPosition(myBody, cpBodyGetPosition(myBody) + cpv(delta[0], delta[1]));
}

SimulationConstraint SimulationSpace::pivotConstrain(SimulationBody bodyA, SimulationBody bodyB, Position anchorPos) const {
    cpVect pos = cpv(anchorPos[0], anchorPos[1]);
    cpConstraint* cons = cpPivotJointNew(bodyA.myBody, bodyB.myBody, pos);
    cpSpaceAddConstraint(mySpace, cons);
    return SimulationConstraint(mySpace, cons, timeStep);
}

SimulationConstraint SimulationSpace::rotarySpringConstrain(SimulationBody bodyA, SimulationBody bodyB, double restAngle, double stiffness, double damping) const {
    cpConstraint* cons = cpDampedRotarySpringNew(bodyA.myBody, bodyB.myBody, restAngle, stiffness,  damping);
    cpSpaceAddConstraint(mySpace, cons);
    return SimulationConstraint(mySpace, cons, timeStep);
}

SimulationSpace::~SimulationSpace() {
    std::cout << "Freeing space..." << std::endl;
    cpSpaceFree(mySpace);
}