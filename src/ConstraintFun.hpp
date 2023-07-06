#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <vector>

class ConstraintFun {
    private:
        int rows;
        int cols;
        std::vector<int> rowConstraints;
        std::vector<int> colConstraints;
    public:
        ConstraintFun(int rows, int cols, std::vector<int> cells);
        std::vector<int> getRowConstraints() {return rowConstraints;};
        std::vector<int> getColConstraints() {return colConstraints;};
};