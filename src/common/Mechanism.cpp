#include "Mechanism.hpp"
#include <unordered_map>
#include <vector>
#include <iostream>

using std::unordered_map;
using std::vector;

void Mechanism::addCell(Cell cell)
{
    cells.push_back(cell);
}

namespace std
{
    template <>
    struct hash<Position>
    {
        size_t operator()(const Position &p) const
        {
            return std::hash<double>()(p[0]) ^ std::hash<double>()(p[1]);
        }
    };
    template <>
    struct hash<Link>
    {
        size_t operator()(const Link &l) const
        {
            return std::hash<Position>()(l.from) ^ std::hash<Position>()(l.to);
        }
    };
}

SimulatedMechanism Mechanism::makeSimulation(float timestep, float linkLength, float linkMass, float linkRadius, float springStiffness, float springDamping, float cornerOffset)
{
    SimulationSpacePtr spacePtr = std::make_shared<SimulationSpace>(timestep);
    const SimulationSpace& space = *spacePtr;
    vector<SimulationBody> linkBodies;
    vector<SimulationConstraint> pivotJoints;
    vector<SimulationConstraint> rotSprings;
    vector<SimulatedCell> simCells;
    std::unordered_map<Link, SimulationBody> existingLinks = {};
    for (auto cell : cells)
    {
        vector<Link> cellLinks;
        vector<double> linkLengths;
        for (int i = 0; i < cell.corners.size(); i++)
        {
            // first pass creates the bodies if necessary
            int next_i = (i + 1) % cell.corners.size();
            Link current(cell.corners[i], cell.corners[next_i]);
            if (existingLinks.find(current) == existingLinks.end() && existingLinks.find(current.reverse()) == existingLinks.end())
            {
                SimulationBody linkBody = space.addSegmentBody(current.getOffsetFrom(cornerOffset), current.getOffsetTo(cornerOffset), linkMass, linkRadius);
                existingLinks[current] = linkBody;
                linkBodies.push_back(linkBody);
            }
            cellLinks.push_back(current);
            linkLengths.push_back(sqrt((current.from[0] - current.to[0]) * (current.from[0] - current.to[0]) + (current.from[1] - current.to[1]) * (current.from[1] - current.to[1])));
        }
        if (cell.isRigid && cell.corners.size() > 3)
        {
            for (int i = 0; i < cell.corners.size(); i += 2)
            {
                // first pass creates the bodies if necessary
                int next_i = (i + 1) % cell.corners.size();
                Link current(cell.corners[i], cell.corners[next_i]);
                if (existingLinks.find(current) == existingLinks.end() && existingLinks.find(current.reverse()) == existingLinks.end())
                {
                    SimulationBody linkBody = space.addSegmentBody(current.getOffsetFrom(cornerOffset), current.getOffsetTo(cornerOffset), linkMass, linkRadius);
                    existingLinks[current] = linkBody;
                    linkBodies.push_back(linkBody);
                }
                cellLinks.push_back(current);
            }
        }
        vector<SimulationBody> cellLinkBodies;
        vector<SimulationConstraint> cellPivots;
        vector<SimulationConstraint> cellRotSprings;
        for (int i = 0; i < cellLinks.size(); i++)
        {
            // second pass constrains the bodies
            int next_i = (i + 1) % cellLinks.size();
            Link current = cellLinks[i];
            Link next = cellLinks[next_i];
            auto currentBodyIt = existingLinks.find(current);
            if (currentBodyIt == existingLinks.end())
                currentBodyIt = existingLinks.find(current.reverse());
            auto nextBodyIt = existingLinks.find(next);
            if (nextBodyIt == existingLinks.end())
                nextBodyIt = existingLinks.find(next.reverse());
            SimulationBody currentBody = currentBodyIt->second;
            SimulationBody nextBody = nextBodyIt->second;
            cellLinkBodies.push_back(currentBody);
            cellPivots.push_back(space.pivotConstrain(currentBody, nextBody, current.to));
            cellRotSprings.push_back(space.rotarySpringConstrain(currentBody, nextBody, 0, 1, 1));
        }
        pivotJoints.insert(pivotJoints.end(), cellPivots.begin(), cellPivots.end());
        rotSprings.insert(rotSprings.end(), cellRotSprings.begin(), cellRotSprings.end());
        SimulatedCell simcell(cell.corners, cellLinkBodies, linkLengths, cellPivots, cellRotSprings);
        simCells.push_back(simcell);
    }
    return SimulatedMechanism(spacePtr, linkBodies, pivotJoints, rotSprings, simCells, linkLength, linkMass, linkRadius, springStiffness, springDamping, cornerOffset);
}

Grid::Grid(int rows, int cols) {
    for(int c_i = 0; c_i < cols; c_i++) {
        for(int r_i = 0; r_i < rows; r_i ++) {
            cells.push_back(Cell({{float(c_i), float(r_i)}, {float(c_i) + 1, float(r_i)}, {float(c_i) + 1, float(r_i) + 1}, {float(c_i), float(r_i) + 1}}));
        }
    }
}

void SimulatedMechanism::step()
{
    space.step();
    for(int i = 0; i < cells.size(); i++) {
        cells[i].update();
    }
}

void SimulatedCell::update()
{
    for(int i = 0; i < corners.size(); i++) {
        Position newCorner =  links[i].getOffsetAlongSegment(-linkLengths[i]);
        // std::cout << "old corner: " << corners[i][0] << ", " << corners[i][1] << ", new corner: " << newCorner[0] << ", " << newCorner[1] << std::endl;;
        corners[i] = newCorner;
    }
}
