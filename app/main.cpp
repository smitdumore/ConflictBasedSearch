#include "cbs/simulator.hpp"
#include "cbs/controller.hpp"
#include <string>
#include <iostream>
#include <thread>
#include <chrono>

int main(int argc, char *argv[]) {

    std::string mapPath = "../maps/corridor_test.yaml";
    std::cout << "Starting CBS simulation with map: " << mapPath << std::endl;

    Controller ctrl;

    // Load map
    std::cout << "Loading map..." << std::endl;
    if (!ctrl.loadMapFromYAML(mapPath)) {
        std::cerr << "Failed to load map!" << std::endl;
        return 1;
    }
    
    // Run First Planner Solution
    if (!ctrl.initializePlanner()) {
        std::cerr << "Failed to initialize Planner" << std::endl;
        return 2;
    }

    if(!ctrl.computeInitialPlan()){
        std::cerr << "Failed to compute Intial Plan" << std::endl;
        return 2;
    }
    
    // Initialize viz Window
    if(!ctrl.initializeVizWindow()) {
        std::cerr << "Failed to create window" << std::endl;
    }
 
    // Run Controller
    std::cout << "Running simulation..." << std::endl;
    std::cout << "The simulation will display in a separate window." << std::endl;
    ctrl.run();
}
