#include "SimulationSpace.hpp"
#include <vector>
#include <unordered_map>

using std::unordered_map;
using std::vector;

class Cell
{
public:
    vector<Position> corners;
    bool isRigid = false;
    Cell(vector<Position> corners) : corners(corners){};
};

class Link
{
public:
    Position from;
    Position to;
    Link(Position from, Position to) : from(from), to(to){};
    bool operator==(Link const &other)
    {
        return other.from == from && other.to == to;
    }
    Link reverse()
    {
        return Link(to, from);
    }
    Position getOffsetFrom(float offset)
    {
        float dist = sqrt((from[0] - to[0]) * (from[0] - to[0]) + (from[1] - to[1]) * (from[1] - to[1]));
        float ratio = offset / dist;
        return {from[0] + ratio * (to[0] - from[0]), from[1] + ratio * (to[1] - from[1])};
    }
    Position getOffsetTo(float offset)
    {
        float dist = sqrt((from[0] - to[0]) * (from[0] - to[0]) + (from[1] - to[1]) * (from[1] - to[1]));
        float ratio = offset / dist;
        return {to[0] - ratio * (to[0] - from[0]), to[1] - ratio * (to[1] - from[1])};
    }
};

class SimulatedCell
{
public:
    vector<SimulationBody> links;
    vector<Position> corners;
    vector<SimulationConstraint> pivotJoints;
    vector<SimulationConstraint> rotarySprings;
    SimulatedCell(vector<Position> corners, vector<SimulationBody> links, vector<SimulationConstraint> pivotJoints, vector<SimulationConstraint> rotarySprings) : corners(corners), links(links), pivotJoints(pivotJoints), rotarySprings(rotarySprings){};
};

class SimulatedMechanism
{
public:
    SimulationSpace space;
    vector<SimulationBody> links;
    vector<SimulationConstraint> pivotJoints;
    vector<SimulationConstraint> rotarySprings;
    vector<SimulatedCell> cells;
    float linkLength, linkMass, linkRadius, springStiffness, springDamping, cornerOffset;
    SimulatedCell getCorrespondingSimulatedCell(Cell cell);
    SimulatedMechanism(SimulationSpace space, vector<SimulationBody> links, vector<SimulationConstraint> pivotJoints, vector<SimulationConstraint> rotarySprings, vector<SimulatedCell> cells, float linkLength, float linkMass, float linkRadius, float springStiffness, float springDamping, float cornerOffset) : space(space), links(links), pivotJoints(pivotJoints), rotarySprings(rotarySprings), cells(cells), linkLength(linkLength), linkMass(linkMass), linkRadius(linkRadius), springStiffness(springStiffness), springDamping(springDamping), cornerOffset(cornerOffset){};
};

class Mechanism
{
public:
    vector<Cell> cells;
    Mechanism() {};
    void addCell(Cell cell);
    SimulatedMechanism makeSimulation(float timestep, float linkLength, float linkMass, float linkRadius, float springStiffness, float springDamping, float cornerOffset);
};

class Grid : public Mechanism
{
public:
    Grid(int rows, int cols);
};