#include "Animation.hpp"
#include "MultiAnimation.hpp"
#include "GridModel.h"
#include "SimuAn.hpp"
#include "SimAnnMan.hpp"
#include <fstream>
#include <float.h>
#include <fstream>
#include <filesystem>
#include <sstream>

#define PI 3.14159265


std::string get_folder_path(std::string file)
{
    auto index = file.find_last_of("/\\");
    auto path = file.substr(0, index + 1);
    return path;
}

std::string get_this_folder_path()
{
    return get_folder_path(__FILE__);
}

std::string parse_string(std::string content)
{
    std::string variable = content;
    //write_log(3) << content << ": " << variable << endl;

    return variable;
}

bool parse_bool(std::string content)
{
    int variable = atoi(content.c_str());
    //write_log(3) << content << ": " << boolalpha << variable << endl;

    return variable;
}

int parse_int(std::string content)
{
    int variable = atoi(content.c_str());
    //write_log(3) << content << ": " << variable << endl;

    return variable;
}



void printConstraintGraph(GridModel gm)
{
  auto cG = gm.constraintGraph;
  std::cout << std::endl
            << "Constraint Graph [" << std::endl;
  int compCount = 0;

  for (auto comp : cG)
  {
    std::cout << "Component " << compCount << std::endl;
    for (auto constraint : comp)
    {
      std::cout << "{";
      std::cout << "Cell: " << constraint.first.vertices[0] << ", " << constraint.first.vertices[1];
      std::cout << ", " << constraint.first.vertices[2] << ", " << constraint.first.vertices[3] << ". Constraints:";
      for (auto edge : constraint.second)
      {
        std::cout << " (" << edge.first << "," << edge.second << ")";
      }
      std::cout << "} ";
    }
    compCount++;
    std::cout << std::endl;
  }
  std::cout << "]" << std::endl;
}

// Returns vector of interior angles of each cell at each time step. By convention, the lower-left angle (vertices 3, 0, 1).
std::vector<std::vector<double> > get_angles(std::vector<GridResult> res, GridModel model)
{
  std::vector<std::vector<double> > angles;
  for (auto frame : res)
  {
    std::vector<double> anglesThisFrame;
    for (auto cell : model.cells)
    {
      Eigen::Vector2d vec1 = frame.points[cell.vertices[1]] - frame.points[cell.vertices[0]];
      Eigen::Vector2d vec2 = frame.points[cell.vertices[3]] - frame.points[cell.vertices[0]];
      double angle = std::atan2(vec1[0] * vec2[1] - vec1[1] * vec2[0], vec1.dot(vec2));
      anglesThisFrame.push_back(angle);
    }
    angles.push_back(anglesThisFrame);
  }
  return angles;
}

void storeModel(GridModel gm, std::string outFolder)
{
  std::ofstream gridOutFile;
  gridOutFile.open(outFolder + "output_model", std::ofstream::out | std::ofstream::trunc);

  gridOutFile << "#num_vertices #num_cells #num_anchors #index_inputvertex #num_inputpoints #index_outputvertex #num_outputpoints\n";
  gridOutFile << gm.points.size() << " " << gm.cells.size() << " " << gm.anchors.size() << " ";
  if (gm.targets.size() != 0)
  {
    gridOutFile << gm.targets[0] << " " << gm.targetPaths[0].size() << " ";
  }
  else
  {
    gridOutFile << "-1 0 ";
  }

  if (gm.inputs.size() != 0)
  {
    gridOutFile << gm.inputs[0] << " " << gm.inputPaths[0].size() << " \n";
  }
  else
  {
    gridOutFile << "-1 0 \n";
  }

  gridOutFile << "\n#vertices\n";
  for (auto p : gm.points)
  {
    gridOutFile << p[0] << " " << p[1] << "\n";
  }

  gridOutFile << "\n#anchors\n";
  for (auto a : gm.anchors)
  {
    gridOutFile << a << " ";
  }
  gridOutFile << "\n";

  gridOutFile << "\n#cells [type s=shear r=rigid a=actuating]\n";
  for (auto c : gm.cells)
  {
    if (c.type == RIGID)
    {
      gridOutFile << "r ";
    }
    else if (c.type == SHEAR)
    {
      gridOutFile << "s ";
    }
    else if (c.type == ACTIVE)
    {
      gridOutFile << "a ";
    }
    gridOutFile << c.vertices[0] << " " << c.vertices[1] << " " << c.vertices[2] << " " << c.vertices[3] << " \n";
  }

  if (gm.targets.size() != 0)
  {
    gridOutFile << "\n#input path\n";
    for (auto p : gm.targetPaths[0])
    {
      gridOutFile << p[0] << " " << p[1] << "\n";
    }
  }

  if (gm.inputs.size() != 0)
  {
    gridOutFile << "\n#output path\n";
    for (auto p : gm.inputPaths[0])
    {
      gridOutFile << p[0] << " " << p[1] << "\n";
    }
  }

  gridOutFile.close();
}

std::vector<std::vector<double> > anglesFromFolder(std::string anglesFolder)
{
  std::vector<std::vector<double> > angles;
  int i = 0;
  while (std::filesystem::exists(anglesFolder + "a" + std::to_string(i)))
  {
    std::ifstream frame(anglesFolder + "a" + std::to_string(i));
    std::vector<double> anglesThisFrame;
    std::string line;
    for (std::getline(frame, line); !line.empty(); std::getline(frame, line))
    {
      anglesThisFrame.push_back(std::stod(line));
    }
    angles.push_back(anglesThisFrame);
    i++;
  }
  return angles;
}

// Some simulation debug code
void helper() {
  // recreate multilayer animation

  // GridModel anim_gm1, anim_gm2, anim_gm_file1, anim_gm_file2;

  // // anim_gm1.loadFromFile("../example-data/inputs/layers/cells_layers_4x4_60.txt");
  // // anim_gm2.loadFromFile("../example-data/inputs/layers/cells_layers_3x3_60.txt");

  // anim_gm1.loadFromFile("../example-data/inputs/cells_quad_complexity.txt");
  // std::string animFolder1 = "../example-data/results/quad_low_complexity/";

  // // anim_gm2.loadFromFile("../example-data/inputs/cells_large_notifier_move_hori.txt");
  // // std::string animFolder2 = "../example-data/results/large_notifier/";

  // // std::string animFolder1 = "../example-data/results/layers/layer_4x4/";
  // std::string anglesFolder1 = animFolder1 + "function_0/angles/";

  // // std::string animFolder2 = "../example-data/results/layers/layer_3x3/";
  // // std::string anglesFolder2 = animFolder2 + "function_1/angles/";

  // anim_gm_file1.loadFromFile(animFolder1 + "output_model");
  // // anim_gm_file2.loadFromFile(animFolder2 + "output_model");

  // auto file_ret1 = optimizeActive(anim_gm_file1, anglesFromFolder(anglesFolder1), "", "");
  // // auto file_ret2 = optimizeActive(anim_gm_file2, anglesFromFolder(anglesFolder2), "", "");

  // std::vector<GridModel> animation_gms;
  // std::vector<std::vector<GridResult>> animation_results;
  // std::vector<std::vector<std::vector<GridModel::Point>>> animation_target_paths;
  // std::vector<std::vector<int>> animation_targets;

  // animation_gms.push_back(anim_gm_file1);
  // // animation_gms.push_back(anim_gm_file2);

  // animation_results.push_back(file_ret1);
  // // animation_results.push_back(file_ret2);

  // animation_target_paths.push_back(anim_gm1.targetPaths);
  // // animation_target_paths.push_back(anim_gm2.targetPaths);

  // animation_targets.push_back(anim_gm1.targets);
  // // animation_targets.push_back(anim_gm2.targets);

  // MultiAnimation test(animation_gms, animation_results, animation_target_paths, 2, animation_targets, false);
  // test.animate_mesh();
  // abort();

  // Using known configuration of gm_active, simulate path given in gm

  //std::string functionFolder = "../example-data/results/rev_eng/water/";
  //std::string pointsFolder = functionFolder + "points/";
  //std::string anglesFolder = functionFolder + "angles/";

  //GridModel gm, gm_active;
  //gm.loadFromFile("../example-data/inputs/rev_eng/water_clean.txt");
  //gm_active.loadFromFile("../example-data/inputs/rev_eng/water.txt");



    // -------------- WORKING VERSION
    //std::string currentFolder = get_this_folder_path();

    //std::string functionFolder = "../example-data/results/rev_eng/water/";
    //std::string pointsFolder = functionFolder + "points/";
    //std::string anglesFolder = functionFolder + "angles/";

    //GridModel gm, gm_active;
    //gm.loadFromFile("../example-data/inputs/UIST2022 (reject)/from_config_and_path/water_clean.txt");
    //gm_active.loadFromFile("../example-data/inputs/UIST2022 (reject)/from_config_and_path/water.txt");

    //auto ret = optimize(gm, "");

    //auto cell_angles = get_angles(ret, gm);
    //auto active_ret = optimizeActive(gm_active, cell_angles, pointsFolder, anglesFolder);

    //// set up animation: pass in as vector
    //std::vector<GridModel> animation_gms;
    //animation_gms.push_back(gm_active);

    //std::vector<std::vector<GridResult>> animation_results;
    //animation_results.push_back(active_ret);

    //std::vector<std::vector<std::vector<GridModel::Point>>> animation_target_paths;
    //animation_target_paths.push_back(gm.targetPaths);

    //std::vector<std::vector<int>> animation_targets;
    //animation_targets.push_back(gm.targets);

    //// run animation
    //MultiAnimation animation(animation_gms, animation_results, animation_target_paths, 2, animation_targets, false);
    //animation.animate_mesh();

    //// Write angles of active cells as csv to new file
    //std::ofstream activeAngleOutFile;
    //activeAngleOutFile.open(anglesFolder + "active.csv", std::ofstream::out | std::ofstream::trunc);
    //std::vector<int> activeCells;
    //std::string delim = "";
    //for (int i = 0; i < gm_active.cells.size(); i++)
    //{
    //    if (gm_active.cells[i].type == ACTIVE)
    //    {
    //        activeCells.push_back(i);
    //        activeAngleOutFile << delim << "Cell " << i;
    //        delim = ",";
    //    }
    //}
    //activeAngleOutFile << "\n";
    //auto angles = anglesFromFolder(anglesFolder);
    //for (auto frame : angles)
    //{
    //    delim = "";
    //    for (int cell : activeCells)
    //    {
    //        activeAngleOutFile << delim << frame[cell];
    //        delim = ",";
    //    }
    //    activeAngleOutFile << "\n";
    //}
    //activeAngleOutFile.close();

    //abort();
    // -------------- end WORKING VERSION







  std::string currentFolder = get_this_folder_path();
  std::string functionFolder = "../example-data/results_local/2022-09-13 (good) 8x8, 3f, 5a - envelope, quarifolium, heart/function_0";
  std::string pointsFolder = functionFolder + "points/";
  std::string anglesFolder = functionFolder + "angles/";

  GridModel gm, gm_active;
  gm.loadFromFile("../example-data/results_local/2022-09-13 (good) 8x8, 3f, 5a - envelope, quarifolium, heart/output_model--passive_for_playback");
  gm_active.loadFromFile("../example-data/results_local/2022-09-13 (good) 8x8, 3f, 5a - envelope, quarifolium, heart/output_model");

  auto ret = optimize(gm, "");

  auto cell_angles = get_angles(ret, gm);
  auto active_ret = optimizeActive(gm_active, cell_angles, pointsFolder, anglesFolder);

  // set up animation: pass in as vector
  std::vector<GridModel> animation_gms;
  animation_gms.push_back(gm_active);

  std::vector<std::vector<GridResult>> animation_results;
  animation_results.push_back(active_ret);

  std::vector<std::vector<std::vector<GridModel::Point>>> animation_target_paths;
  animation_target_paths.push_back(gm.targetPaths);

  std::vector<std::vector<int>> animation_targets;
  animation_targets.push_back(gm.targets);

  // run animation
  MultiAnimation animation(animation_gms, animation_results, animation_target_paths, 2, animation_targets, false); 
  animation.animate_mesh();                                                     

  // Write angles of active cells as csv to new file
  std::ofstream activeAngleOutFile;
  activeAngleOutFile.open(anglesFolder + "active.csv", std::ofstream::out | std::ofstream::trunc);
  std::vector<int> activeCells;
  std::string delim = "";
  for (int i = 0; i < gm_active.cells.size(); i++)
  {
    if (gm_active.cells[i].type == ACTIVE)
    {
      activeCells.push_back(i);
      activeAngleOutFile << delim << "Cell " << i;
      delim = ",";
    }
  }
  activeAngleOutFile << "\n";
  auto angles = anglesFromFolder(anglesFolder);
  for (auto frame : angles)
  {
    delim = "";
    for (int cell : activeCells)
    {
      activeAngleOutFile << delim << frame[cell];
      delim = ",";
    }
    activeAngleOutFile << "\n";
  }
  activeAngleOutFile.close();

  abort();
}

int main(int argc, char *argv[])
{ 
   //helper(); //uncomment to run helper

    /*
  // command line interface
  std::string strInput, output_folder;
  int num_layers, num_iters;
  double path_weight, dof_weight;
  bool ground;

  // get number of layers
  while (true)
  {
    std::cout << "Please enter the number of functions: ";
    std::getline (std::cin, strInput);

    std::stringstream inputStream(strInput);
    if ((inputStream >> num_layers)) {break;}
    std::cout << "Invalid input, please input an integer." << std::endl;
  }

  // get grids
  std::vector<GridModel> gms;

  std::cout << "Note: all functions must have same dimension." << std::endl;

  for (int i = 0; i < num_layers; i++)
  {
    GridModel gm;
    while (true)
    {
      std::cout << "Please input path to function " << i << "'s file: ";
      std::getline (std::cin, strInput);

      if (gm.loadFromFile(strInput)) {break;}
    }
    gms.push_back(gm);
  }

  // get output folder
  std::cout << "Please specify output folder, with trailing \"\\\": ";
  std::getline (std::cin, output_folder);
  std::filesystem::create_directories(output_folder);

  // get simulated annealing parameters

  // path accuracy weight
  while (true)
  {
    std::cout << "Please enter path accuracy weight: ";
    std::getline (std::cin, strInput);

    std::stringstream inputStream(strInput);
    if ((inputStream >> path_weight)) {break;}
    std::cout << "Invalid input, please input a double" << std::endl;
  }

  // dofs weight
  while (true)
  {
    std::cout << "Please enter complexity weight: ";
    std::getline (std::cin, strInput);

    std::stringstream inputStream(strInput);
    if ((inputStream >> dof_weight)) {break;}
    std::cout << "Invalid input, please input a double." << std::endl;
  }

  // number iterations
  while (true)
  {
    std::cout << "Please enter the number of optimization iterations: ";
    std::getline (std::cin, strInput);

    std::stringstream inputStream(strInput);
    if ((inputStream >> num_iters)) {break;}
    std::cout << "Invalid input, please input an integer." << std::endl;
  }

  // visualize with ground
  while (true)
  {
    std::cout << "Do you want a ground in the visualization? (y/n): ";
    std::getline (std::cin, strInput);

    if (strInput == "y") {
      ground = true;
      break;
    } else if (strInput == "n") {
      ground = false;
      break;
    }
    std::cout << "Invalid input, y/n only." << std::endl;
  }
  */

  // command line interface
    //std::string strInput;
    std::vector<std::string> layer_configs;
    std::string output_folder;
    //int num_layers;
    int num_iters;
    double path_weight, dof_weight;
    bool ground;



    // Open the File
    std::string config_filepath = get_this_folder_path() + "../example-data/config.txt";
    std::ifstream in(config_filepath.c_str());

    // Check if object is valid
    if (!in)
    {
        std::cerr << "Cannot open the STARTUP config file : " << config_filepath << std::endl;
        exit(1);
    }

    std::string line;
    // Read the next line from File untill it reaches the end.
    while (std::getline(in, line))
    {
        if (line.find('#') == 0)
            continue;
        if (line.empty())
            continue;

        auto colon_index = line.find_first_of(":");
        std::string qualifier = line.substr(0, colon_index);
        std::string content = line.substr(colon_index + 2, line.length() - colon_index + 2);

        if (content.empty())
            continue;

        //if (qualifier == "number_functions")
        //{
        //    num_layers = parse_int(content);
        //    std::cout << "num_layers: " << num_layers << std::endl;
        //}
        //else 
        if(qualifier == "function")
        {
            //std::string function = parse_string(content);
            //layer_configs.push_back(function);

            layer_configs.push_back(content);
            std::cout << "function: " << content << std::endl;
        }
        else if (qualifier == "output_folder")
        {
            output_folder = content;
            std::cout << "output_folder: " << content << std::endl;
        }
        else if (qualifier == "number_iterations")
        {
            num_iters = parse_int(content);
            std::cout << "num_iters: " << num_iters << std::endl;
        }
        else if (qualifier == "accuracy_weight")
        {
            path_weight = parse_int(content);
            std::cout << "path_weight: " << path_weight << std::endl;
        }
        else if (qualifier == "complexity_weight")
        {
            dof_weight = parse_int(content);
            std::cout << "dof_weight: " << dof_weight << std::endl;
        }
        else if (qualifier == "show_ground")
        {
            ground = parse_bool(content);
            std::cout << "ground: " << ground << std::endl;
        }
    }

    //Close The File
    in.close();




  // get grids
    std::vector<GridModel> gms;

    //std::cout << "Note: all functions must have same dimension." << std::endl;
    //
    //for (int i = 0; i < num_layers; i++)
    //{
    //    GridModel gm;
    //    //while (true)
    //    //{
    //    //    std::cout << "Please input path to function " << i << "'s file: ";
    //    //    std::getline(std::cin, strInput);
    //
    //    //    if (gm.loadFromFile(strInput)) { break; }
    //    //}
    //
    //}

    for (int i = 0; i < layer_configs.size(); i++)
    {
        GridModel gm;

        bool success = gm.loadFromFile(layer_configs[i]);
        if (!success) 
            break; 

        gms.push_back(gm);
    }

    if (!std::filesystem::exists(output_folder))
        std::filesystem::create_directory(output_folder);

  SimAnnMan sa(gms, output_folder, path_weight, dof_weight);            // Initialize simulated annealing, specifying output folder
  sa.runSimulatedAnnealing(num_iters, 0.97); // Run simulated annealing
  //sa.runSimulatedAnnealing(100, 0.97); // Run simulated annealing

  gms = sa.bestModels;           // Get best model from simulated annealing

  GridModel gm_active_base = GridModel(gms[0]).addActiveCells(); // add active cells
  std::vector<GridModel> combined_animation_gms;
  std::vector<std::vector<GridResult>> combined_results;
  std::vector<std::vector<std::vector<GridModel::Point>>> combined_target_paths;
  std::vector<std::vector<int>> combined_targets;

  for (int i = 0; i < gms.size(); i++) {
    GridModel gm = GridModel(gms[i]);

    std::string functionFolder = output_folder + "function_" + std::to_string(i) + "/";
    std::string pointsFolder = functionFolder + "points/";
    std::string anglesFolder = functionFolder + "angles/";

    std::filesystem::create_directory(functionFolder);
    std::filesystem::create_directory(pointsFolder);
    std::filesystem::create_directory(anglesFolder);


    auto ret = optimize(gm, ""); // run optimize to get grid position at each frame

    auto cell_angles = get_angles(ret, gm);                                               // get angles for each cell at each frame
    GridModel gm_active = GridModel(gm);
    gm_active.cells = gm_active_base.cells;                                               // add active cells
    auto active_ret = optimizeActive(gm_active, cell_angles, pointsFolder, anglesFolder); // Call optimizeActive to verify results with only control of actuating cells
    storeModel(gm_active, output_folder);                                                        // Store best model with actuating cells in results folder

    // set up animation: pass in as vector
    std::vector<GridModel> animation_gms;
    animation_gms.push_back(gm_active);
    combined_animation_gms.push_back(gm_active);

    std::vector<std::vector<GridResult>> animation_results;
    animation_results.push_back(active_ret);
    combined_results.push_back(active_ret);

    std::vector<std::vector<std::vector<GridModel::Point>>> animation_target_paths;
    animation_target_paths.push_back(gm.targetPaths);
    combined_target_paths.push_back(gm.targetPaths);

    std::vector<std::vector<int>> animation_targets;
    animation_targets.push_back(gm.targets);
    combined_targets.push_back(gm.targets);

    // run animation
    MultiAnimation animation(animation_gms, animation_results, animation_target_paths, 2, animation_targets, ground); 
    animation.animate_mesh();                                                       

    // Write angles of active cells as csv to new file
    std::ofstream activeAngleOutFile;
    activeAngleOutFile.open(anglesFolder + "active.csv", std::ofstream::out | std::ofstream::trunc);
    std::vector<int> activeCells;
    std::string delim = "";
    for (int i = 0; i < gm_active.cells.size(); i++)
    {
      if (gm_active.cells[i].type == ACTIVE)
      {
        activeCells.push_back(i);
        activeAngleOutFile << delim << "Cell " << i;
        delim = ",";
      }
    }
    activeAngleOutFile << "\n";
    auto angles = anglesFromFolder(anglesFolder);
    for (auto frame : angles)
    {
      delim = "";
      for (int cell : activeCells)
      {
        activeAngleOutFile << delim << frame[cell];
        delim = ",";
      }
      activeAngleOutFile << "\n";
    }
    activeAngleOutFile.close();
  }

  // run combined
  MultiAnimation combinedAnimation(combined_animation_gms, combined_results, combined_target_paths, 2, combined_targets, ground);
  combinedAnimation.animate_mesh();
  

  // Verify everything works by constructing gridmodel and angles from files
  // GridModel gmFile;
  // gmFile.loadFromFile(folder + "output_model");
  // auto file_ret = optimizeActive(gmFile, anglesFromFolder(anglesFolder), "", "");
  // Animation test(gmFile, file_ret, gm.targetPaths, 2, gm.targets);
  // test.animate();
}
