#include "UIModelData.hpp"

int UIModelData::selectedCell = 0;
int UIModelData::selectedJoint = 0;
std::vector<int> UIModelData::modelDimensions = { 1,1 };
std::vector<int> UIModelData::cells = { 0 };
bool UIModelData::dimChanged = false;
bool UIModelData::cellsEdited = false;

std::vector<MMGrid> UIModelData::gridSet = { MMGrid(UIModelData::modelDimensions[0], UIModelData::modelDimensions[1], UIModelData::cells) };
int UIModelData::gridIndex = 0;


bool UIModelData::joint_path_editor_visible = false;
bool UIModelData::playback_options_visible = false;
bool UIModelData::opt_wseights_visible = false;
bool UIModelData::sim_params_visible = false;
bool UIModelData::sim_info_visible = false;

bool UIModelData::sim_running = true;
bool UIModelData::playing = false;

float UIModelData::simTimestep = 0.01;
float UIModelData::playbackPointsPerSecond = 2;

int UIModelData::annealingSteps = 20;
float UIModelData::pathWeight = 2.0;
float UIModelData::dofWeight = 3.0;

std::unordered_map<std::string, std::vector<cpVect>> UIModelData::paths = {};
string UIModelData::pathSelection = "";
float UIModelData::pathScale = 1.0;

vector<vector<vector<cpVect>>> UIModelData::allCalculatedPaths = {};