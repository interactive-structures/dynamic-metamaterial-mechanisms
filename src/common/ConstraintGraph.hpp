#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <vector>

using std::vector;

class ConstraintGraph
{
private:
    int rows;
    int cols;
    std::vector<int> rowConstraints;
    std::vector<int> colConstraints;
    std::vector<int> allConstraints;
    void tieRC(int rowIndex, int colIndex);
    bool allConstraintsUpdated = false;
    void updateAllConstraints();

public:
    ConstraintGraph(int rows, int cols, vector<int> cells);
    ConstraintGraph(vector<int> rowConstraints, vector<int> colConstraints);
    std::vector<int> getRowConstraints() { return rowConstraints; };
    std::vector<int> getColConstraints() { return colConstraints; };
    std::vector<int> getAllConstraints()
    {
        if (!allConstraintsUpdated)
            updateAllConstraints();
        return allConstraints;
    }
    int dofs();
    void mergeComponents();
    void splitComponents();
    vector<int> makeCells();
    vector<int> makeCellIndices();
    vector<int> allConstrainedIndices();
    vector<int> allConstrainedCells();
    vector<int> getActiveCellIndices(vector<int> cells);
};