
#include <fstream>
#include <iostream>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>

#include "../include/cbs/cbs.hpp"


int main(int argc, char* argv[]) {
  
  // Declare the supported options.
  // Terminal stuff to run code
  namespace po = boost::program_options;
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  
  bool disappearAtGoal;
  int maxNodes;
  int spaceSlack;
  
  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input file (YAML)")("output,o",
                           po::value<std::string>(&outputFile)->required(),
                           "output file (YAML)")(
      "disappear-at-goal", po::bool_switch(&disappearAtGoal), 
      "make agents to disappear at goal rather than staying there")(
      "max-nodes", po::value<int>(&maxNodes)->default_value(10000),
      "maximum number of high level nodes to explore in CBS")(
      "space-slack", po::value<int>(&spaceSlack)->default_value(2),
      "minimum space between agents in grid cells");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  YAML::Node config = YAML::LoadFile(inputFile);

  // CBS logic start
  std::unordered_set<Location> obstacles;
  std::vector<Location> goals;
  std::vector<State> startStates;

  const auto& dim = config["map"]["dimensions"];
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();

  for (const auto& node : config["map"]["obstacles"]) {
    obstacles.insert(Location(node[0].as<int>(), node[1].as<int>()));
  }

  for (const auto& node : config["agents"]) {
    const auto& start = node["start"];
    const auto& goal = node["goal"];
    startStates.emplace_back(State(0, start[0].as<int>(), start[1].as<int>()));
    goals.emplace_back(Location(goal[0].as<int>(), goal[1].as<int>()));
  }

  // sanity check: no identical start states
  std::unordered_set<State> startStatesSet;
  for (const auto& s : startStates) {
    if (startStatesSet.find(s) != startStatesSet.end()) {
      std::cout << "Identical start states detected -> no solution!" << std::endl;
      return 0;
    }
    startStatesSet.insert(s);
  }

  // Initialize environment
  Environment mapf(dimx, dimy, obstacles, goals, disappearAtGoal, spaceSlack);

  // Initialize CBS templated object with max nodes parameter
  CBS<State, Action, int, Conflict, Constraints, Environment> cbs(mapf, maxNodes);
  std::vector<PlanResult<State, Action, int> > solution;

  bool success = cbs.search(startStates, solution);

  if (success) {
    std::cout << "Planning successful! " << std::endl;
    int cost = 0;
    int makespan = 0; // time take for the entire MAPF problem to execute
    for (const auto& s : solution) {
      cost += s.cost;
      makespan = std::max<int>(makespan, s.cost);
    }

    // Write solution to a file
    std::ofstream out(outputFile);
    out << "statistics:" << std::endl;
    out << "  cost: " << cost << std::endl;
    out << "  makespan: " << makespan << std::endl;
    out << "  highLevelExpanded: " << mapf.highLevelExpanded() << std::endl;
    out << "  lowLevelExpanded: " << mapf.lowLevelExpanded() << std::endl;
    out << "  spaceSlack: " << spaceSlack << std::endl;
    out << "schedule:" << std::endl;
    for (size_t a = 0; a < solution.size(); ++a) {

      out << "  agent" << a << ":" << std::endl;
      for (const auto& state : solution[a].states) {
        out << "    - x: " << state.first.x << std::endl
            << "      y: " << state.first.y << std::endl
            << "      t: " << state.second << std::endl;
      }
    }

    // print cost and makespan
  } else {
    std::cout << "Planning NOT successful!" << std::endl;
  }

  return 0;
}
