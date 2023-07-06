#include "ConstraintFun.hpp"

using std::vector;
ConstraintFun::ConstraintFun(int rows, int cols, vector<int> cells) : rows(rows), cols(cols)
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
            int existingConstraint = colConstraints[colIndex];
            if (existingConstraint == 0) // column is unconstrained
            { 
                colConstraints[colIndex] = rowConstraints[rowIndex];
            }
            else
            {
                if (existingConstraint < rowConstraints[rowIndex]) // column already has lower constraint than the row it's being constrained to
                {
                    //tie all columns tied to previous row to this column's constraint
                    for (int c = 0; c < cols; c++)
                    {
                        if (colConstraints[c] == rowConstraints[rowIndex])
                        {
                            colConstraints[c] = existingConstraint;
                        }
                    }
                    rowConstraints[rowIndex] = existingConstraint;
                }
                else if (existingConstraint > rowConstraints[rowIndex]) // column has higher constraint than the row it's being constrained to
                {
                    //column needs to be tied to this new row
                    rowConstraints[existingConstraint - 1] = rowConstraints[rowIndex];
                    for (int c = 0; c < cols; c++)
                    {
                        if (colConstraints[c] == existingConstraint)
                        {
                            colConstraints[c] = rowConstraints[rowIndex];
                        }
                    }
                }
            }
        }
    }
}