#include "GridModel.h"
#include "MetaGrid.hpp"
#include "coin/IpIpoptApplication.hpp"
#include "coin/IpSolveStatistics.hpp"
#include "MyNLP.hpp"

#include <fstream>

namespace {
    int optimize(SmartPtr<MyNLP>& mynlp)
    {
        sort(mynlp->fixedDofs.begin(), mynlp->fixedDofs.end());
        SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
        
        // Initialize the IpoptApplication and process the options
        ApplicationReturnStatus status;
        status = app->Initialize();
        
        if (status != Solve_Succeeded) {
            std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
            return (int) status;
        }
        
        //  app->Options()->SetNumericValue("bound_mult_reset_threshold", 100);
        //   app->Options()->SetNumericValue("bound_push", 1e-4);
        
        app->Options()->SetIntegerValue("print_level", 5);
        app->Options()->SetStringValue("derivative_test", "second-order");
   //   app->Options()->SetStringValue("derivative_test", "first-order");
        
        
        status = app->OptimizeTNLP(mynlp);
        
        
        /*
         if (status == Solve_Succeeded) {
         // Retrieve some statistics about the solve
         Index iter_count = app->Statistics()->IterationCount();
         std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;
         
         Number final_obj = app->Statistics()->FinalObjective();
         std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.' << std::endl;
         }
         */
        
        mynlp->constrId.clear();
        mynlp->constrTarget.clear();
        mynlp->fixedDofs.clear();
        
        return (int) status;
    }
}

GridCell::GridCell(const int a, const int b, const int c, const int d, CellType t)
: vertices(a, b, c, d), type(t)
{
}


void GridModel::loadFromFile(const std::string fname)
{
    using namespace std;
    ifstream file(fname);
    
    if(!file.good())
    {
        cout << "file " << fname << " not found!" << endl;
        return;
    }
    
    file >> nrows;
    file >> ncols;
    
    int nAnchors;
    
    file >> nAnchors;
    file >> root;
    file >> driver;
    
    int nPath;
    
    file >> nPath;
    
    
    anchors.push_back(root);
    
    for(int i = 0; i < nAnchors; ++i)
    {
        int a;
        file >> a;
        anchors.push_back(a);
    }
    
 
    ++nAnchors;
    
    vector<double> rigidity;
    
    for(int i = 0; i < (nrows ) * (ncols ); ++i)
    {
        double r;
        file >> r;
        rigidity.push_back(r);
    }
    
    inputs.push_back(driver);
    
    std::vector<Point> path;
    
    for(int i = 0; i < nPath; ++i)
    {
        double x, y;
        file >> x;
        file >> y;
        path.push_back(Point(x, y));
    }
    
    inputPaths.push_back(move(path));
    
    // set points and cells
    for(int i = 0; i < ncols + 1; ++i)
        for(int j = 0; j < nrows + 1; ++j)
        {
            points.push_back(Point(i, j));
        }
    
    auto itr = rigidity.begin();
    
    
    for(int i = 0; i < ncols; ++i)
        for(int j = 0; j < nrows; ++j)
        {
            cells.push_back(GridCell(i * (ncols + 1) + j,
                                     (i+1) * (ncols + 1) + j,
                                     (i+1) * (ncols + 1) + (j+1),
                                     i * (ncols + 1) + (j+1),
                                     *itr++ < 1 ? SHEAR : RIGID));
        }
    
    {
        ofstream file("../out");
        for(auto p : points)
            file << p[0] << " " << p[1] << endl;
        
        for(auto& c : cells)
            file << c.vertices[0] << " " << c.vertices[1] << " " << c.vertices[2] << " " << c.vertices[3] << endl;
        
        file.close();
    
    }
}

/*
void GridModel::loadFromFile(const std::string fname)
{
    using namespace std;
    ifstream file(fname);
    
    if(!file.good())
    {
        cout << "file " << fname << " not found!" << endl;
        return;
    }
    char tmp[1024];
    
    file.getline(tmp, 1024);
    
    int nv, nc, na, constr, npath, constr2 = -1, npath2;
    
    file >> nv;
    file >> nc;
    file >> na;
    file >> constr;
    file >> npath;
    
    if(file.peek() != '\n')
    {
        file >> constr2;
        file >> npath2;
    }
    
    if(constr2 != -1) inputs.push_back(constr2);
    targets.push_back(constr);
    
    file.getline(tmp, 1024);
    file.getline(tmp, 1024);
    file.getline(tmp, 1024);
    
    for(int i = 0; i < nv; ++i)
    {
        double x, y;
        file >> x;
        file >> y;
        
        points.push_back(Point(x,y));
    }
    
    file.getline(tmp, 1024);
    file.getline(tmp, 1024);
    file.getline(tmp, 1024);
    
    for(int i = 0; i < na; ++i)
    {
        int x;
        file >> x;
        
        anchors.push_back(x);
    }
    
   // shift = vertices[anchors.front()];
    //  anchors.pop_back();
    
    file.getline(tmp, 1024);
    file.getline(tmp, 1024);
    file.getline(tmp, 1024);
    
    for(int i = 0; i < nc; ++i)
    {
        char t;
        file >> t;
        
        int a,b,c,d;
        file >> a;
        file >> b;
        file >> c;
        file >> d;
        
        cells.push_back(GridCell(a, b, c, d, t == 's' ? SHEAR : RIGID));
    }
    
    file.getline(tmp, 1024);
    file.getline(tmp, 1024);
    file.getline(tmp, 1024);
    
    vector<Point> path;
    
    for(int i = 0; i < npath; ++i)
    {
        double x, y;
        file >> x;
        file >> y;
        
        path.push_back(Point(x,y));
    }
    
    targetPaths.push_back(path);
    path.clear();
    
    if(constr2 != -1)
    {
        file.getline(tmp, 1024);
        file.getline(tmp, 1024);
        file.getline(tmp, 1024);
        
        for(int i = 0; i < npath2; ++i)
        {
            double x, y;
            file >> x;
            file >> y;
            
            path.push_back(Point(x,y));
        }
        
        inputPaths.push_back(path);
    }
    
    file.close();
}*/

std::vector<GridResult>
optimize(const GridModel& model, std::string pointDirectory, bool elastic)
{
    using namespace std;
    
    MetaGrid grid(model);
    grid.setEdges();
    grid.setEdgeRelations();
    
    std::vector<GridResult> ret;
    
    vector<int> sizes;
    for(auto& x : grid.constrained) 
		sizes.push_back(x.second.size());

    int nframes = *min_element(sizes.begin(), sizes.end());
    
    double totError = .0;
    grid.setEdgeRelations();
    auto dofs = grid.degreesOfFreedom();
    
    SmartPtr<MyNLP> mynlp = new MyNLP(grid);
    mynlp->setStartConfiguration(grid.vertices);
    
    mynlp->mode = elastic ? MyNLP::ELASTIC : MyNLP::COMBINATORIC;
    
    int cnt = 0;

    for(int i = 0; i < nframes; i += 1)
    {
        for(auto& x : grid.constrained)
        {
            mynlp->setConstraint(x.first, x.second[i]);
        }
        
        optimize(mynlp);
        totError += mynlp->objError;
        
        auto pts = grid.setDOFs(dofs, mynlp->solution);
        mynlp->setStartConfiguration(pts);
        
        GridResult resi;
        
        for(auto& p : pts) resi.points.push_back(p + grid.shift);
        
      //  resi.points = model.points;
        
        if(!pointDirectory.empty())
        {
            ofstream file(pointDirectory + "/p" + to_string(i));
            for(auto& p : resi.points)
                file << p[0] << " " << p[1] << "\n";
            file.close();
        }
        
        ret.push_back(resi);
    }
    
    return ret;
};
