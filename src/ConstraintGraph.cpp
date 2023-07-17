#include "ConstraintGraph.hpp"
#include <algorithm>
#include "PRand.hpp"

using std::vector;
ConstraintGraph::ConstraintGraph(int rows, int cols, vector<int> cells) : rows(rows), cols(cols)
{
    rowConstraints.resize(rows);
    colConstraints.resize(cols);
    for (int i = 0; i < rows; i++)
    {
        rowConstraints[i] = i + 1;
    }
    for (int i = 0; i < rows * cols; i++)
    {
        int rowIndex = i / cols;
        int colIndex = i % cols;
        if (cells[i] == 1) // cell is rigid
        {
            tieRC(rowIndex, colIndex);
        }
    }
    updateAllConstraints();    
}

void ConstraintGraph::updateAllConstraints() {
    allConstraints.clear();
    allConstraints.reserve(rows + cols);
    allConstraints.insert(allConstraints.end(), rowConstraints.begin(), rowConstraints.end());
    allConstraints.insert(allConstraints.end(), colConstraints.begin(), colConstraints.end());
    allConstraintsUpdated = true;
}

void ConstraintGraph::tieRC(int rowIndex, int colIndex)
{
    allConstraintsUpdated = false;
    int existingConstraint = colConstraints[colIndex];
    if (existingConstraint == 0) // column is unconstrained
    {
        colConstraints[colIndex] = rowConstraints[rowIndex];
    }
    else
    {
        if (existingConstraint < rowConstraints[rowIndex]) // column already has lower constraint than the row it's being constrained to
        {
            // tie all columns tied to previous row to this column's constraint
            for (int c = 0; c < cols; c++)
            {
                if (colConstraints[c] == rowConstraints[rowIndex])
                {
                    colConstraints[c] = existingConstraint;
                }
            }
            int oldRowConstraint = rowConstraints[rowIndex];
            // tie all columns tied to previous row to this column's constraint
            for (int r = 0; r < rows; r++)
            {
                if (rowConstraints[r] == oldRowConstraint)
                {
                    rowConstraints[r] = existingConstraint;
                }
            }
        }
        else if (existingConstraint > rowConstraints[rowIndex]) // column has higher constraint than the row it's being constrained to
        {
            // column needs to be tied to this new row
            rowConstraints[existingConstraint - 1] = rowConstraints[rowIndex];
            for (int c = 0; c < cols; c++)
            {
                if (colConstraints[c] == existingConstraint)
                {
                    colConstraints[c] = rowConstraints[rowIndex];
                }
            }
            // tie all columns tied to previous row to this column's constraint
            for (int r = 0; r < rows; r++)
            {
                if (rowConstraints[r] == existingConstraint)
                {
                    rowConstraints[r] = rowConstraints[rowIndex];
                }
            }
        }
    }
}

void ConstraintGraph::mergeComponents()
{
    PRand colRand(cols);
    for (int c_try = 0; c_try < cols; c_try++)
    {
        int c = colRand.next();
        PRand rowRand(rows);
        for (int r_try = 0; r_try < rows; r_try++)
        {
            int r = rowRand.next();
            if(rowConstraints[r] != colConstraints[c]) { //rowConstraints[colConstraints[c] - 1] == colConstraints[c], presumably (if constraints set up correctly), and colConstraints[c] == 0 will be true if the expression in the if statement is true 
                tieRC(r, c);
                return;
            }
        }
    }
}

void ConstraintGraph::splitComponents()
{
    vector<int> cellIndices = makeCellIndices();
    vector<int> cells(rows*cols);
    int skip = cellIndices[rand() % cellIndices.size()];
    for (int i = 0; i < rows; i++)
    {
        rowConstraints[i] = i + 1;
    }
    colConstraints = std::vector<int>(cols);
    for(int rigidIndex : cellIndices) {
        std::cout << rigidIndex << " " << skip << std::endl;
        if(rigidIndex != skip) {
            int rowIndex = rigidIndex / cols;
            int colIndex = rigidIndex % cols;
            tieRC(rowIndex, colIndex);
        }
    }
}

int ConstraintGraph::dofs()
{
    int dofs = 0;
    vector<int> allCs = allConstraints;
    std::sort(allCs.begin(), allCs.end());
    dofs += std::unique(allCs.begin(), allCs.end()) - allCs.begin();
    int zcount = std::count(colConstraints.begin(), colConstraints.end(), 0);
    dofs += zcount - (zcount == 0 ? 0 : 1); // if there are any zeros, get rid of the overlap with the one in the unique count
    return dofs;
}

vector<int> ConstraintGraph::makeCellIndices() {
    //this should return a minimal (non-redundant) configuration of rigid cells to get the current constraint graph
    vector<int> result;
    int n = rows*cols;
    vector<int> blankCells(n);
    ConstraintGraph other(rows, cols, blankCells);
    PRand cellRand(n);
    for(int i = 0; i < n; i++) {
        int cellIndex = cellRand.next();
        int rowIndex = cellIndex / cols;
        int colIndex = cellIndex % cols;
        // std::cout << "Trying cell (" << rowIndex << ", " << colIndex << ")..." << std::endl;
        if(rowConstraints[rowIndex] == colConstraints[colIndex] && other.rowConstraints[rowIndex] != other.colConstraints[colIndex]) {
            other.tieRC(rowIndex, colIndex);
            // std::cout << "We got:" << std::endl;
            // for(auto c : rowConstraints) std::cout << c;
            // for(auto c : colConstraints) std::cout << c;
            // std::cout << std::endl;
            // for(auto c : other.rowConstraints) std::cout << c;
            // for(auto c : other.colConstraints) std::cout << c;
            // std::cout << std::endl;
            result.push_back(cellIndex);
            if(other.rowConstraints == rowConstraints && other.colConstraints == colConstraints) {
                // for(auto c : rowConstraints) std::cout << c;
                // for(auto c : colConstraints) std::cout << c;
                // std::cout << std::endl;
                // for(auto c : other.rowConstraints) std::cout << c;
                // for(auto c : other.colConstraints) std::cout << c;
                // std::cout << std::endl;
                return result;
            }
        }
    }
    // std::cout << "This should have been uncreachable..." << std::endl;
    return result;
}

vector<int> ConstraintGraph::makeCells() {
    vector<int> result(rows*cols);
    for(int rigidIndex : makeCellIndices()) {
        result[rigidIndex] = 1;
    }
    return result;
}

vector<int> ConstraintGraph::allConstrainedIndices() {
    //this should return a maximal (redundant) configuration of rigid cells to get the current constraint graph
    vector<int> result;
    int n = rows*cols;
    vector<int> blankCells(n);
    ConstraintGraph other(rows, cols, blankCells);
    PRand cellRand(n);
    for(int i = 0; i < n; i++) {
        int cellIndex = cellRand.next();
        int rowIndex = cellIndex / cols;
        int colIndex = cellIndex % cols;
        if(rowConstraints[rowIndex] == colConstraints[colIndex]) {
            other.tieRC(rowIndex, colIndex);
            result.push_back(cellIndex);
        }
    }
    // std::cout << "This should have been uncreachable..." << std::endl;
    return result;
}

vector<int> ConstraintGraph::allConstrainedCells() {
    vector<int> result(rows*cols);
    for(int rigidIndex : allConstrainedIndices()) {
        result[rigidIndex] = 1;
    }
    return result;
}