#include "GridModel.h"

int main(int argv, char* argc[])
{
    GridModel gm;
    gm.loadFromFile("../cells.txt");
    auto ret = optimize(gm, "../points/");
  
    return 0;
}
