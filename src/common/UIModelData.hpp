#include <vector>
#include "MMGrid.hpp"

#pragma once
class UIModelData
{
public:
	static MMGrid& modelGrid() {
		return gridSet[gridIndex];
	};
	static std::vector<MMGrid> gridSet;
	static int gridIndex;

	static int selectedCell;
	static int selectedJoint;
	static std::vector<int> modelDimensions;
	static std::vector<int> cells;
	static bool dimChanged;
	static bool cellsEdited;

	static bool joint_path_editor_visible;
	static bool playback_options_visible;
	static bool opt_wseights_visible;
	static bool sim_params_visible;
	static bool sim_info_visible;

	static bool sim_running;
	static bool playing;

	static float simTimestep;
	static float playbackPointsPerSecond;

	static int annealingSteps;
	static float pathWeight;
	static float dofWeight;

	static std::unordered_map<std::string, std::vector<cpVect>> paths;
	static string pathSelection;
	static float pathScale;

	static vector<vector<vector<cpVect>>> allCalculatedPaths;
};