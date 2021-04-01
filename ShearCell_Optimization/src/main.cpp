#include <iostream>

#include "GridModel.h"
#include "MetaGrid.hpp"
#include "Optimizer.hpp"
#include "coin/IpIpoptApplication.hpp"
#include "MyNLP.hpp"

using namespace std;

template<class TOptimizer>
Eigen::VectorXd run(SmartPtr<TOptimizer>& opt)
{
    using namespace Ipopt;
    
    cout << "####################################################" << endl;
    
    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
    app->Options()->SetIntegerValue("print_level", 3);
    app->Options()->SetStringValue("derivative_test", "second-order");
    
    // Intialize the IpoptApplication and process the options
    ApplicationReturnStatus status;
    status = app->Initialize();
    
    if (status != Solve_Succeeded) {
        printf("\n\n*** Error during initialization!\n");
        return;
    }
    
    // set target
    //   targetPosition(0) = 0;
    //    targetPosition(1) = 3;
    
    
    // Ask Ipopt to solve the problem

    status = app->OptimizeTNLP(opt);
   
    cout << "####################################################" << endl;
    
    
    return opt->solution;
}

void run()
{
    GridModel gmodel;
    gmodel.loadFromFile("../gridArcToCorner2");
    
    optimize(gmodel, "../points2/", true);
}

int main(int argv, char* argc[])
{
    run();
    return 0;
    
    SimulationGrid grid;
    grid.loadFromFile("../grid");

    SmartPtr<Optimizer> opt = new Optimizer(grid);
    
    for(int i = 0; i < grid.driverPath.size(); ++i)
    {
        opt->setTarget(grid.driverPath[i]);
       // opt->setTarget(Eigen::Vector2d(.01 * i,  .01 * i));
        
        // first pass: find closest feasable point to target
        opt->targetHardConstrained = false;
        auto sol1 = run(opt);
        
        // second pass: now optimize for the correct physical deformation
        opt->targetHardConstrained = true;
        opt->setTarget(opt->getTargetPositon());
        auto sol = run(opt);
        
        opt->setStartingPoint(sol);
        grid.writeResult("../points/out" + to_string(i), sol);
    }
    
    return 0;
}
