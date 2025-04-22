#include <QApplication>
#include "cbs/simulator.hpp"
#include "cbs/ui.hpp"
#include <string>
#include <iostream>
#include <thread>
#include <chrono>

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    std::string mapPath = "../maps/corridor_test.yaml";
    std::cout << "Starting CBS simulation with map: " << mapPath << std::endl;

    // Create UI instance instead of simulation
    UI ui;

    // Load map
    std::cout << "Loading map..." << std::endl;
    if (!ui.loadMapFromYAML(mapPath)) {
        std::cerr << "Failed to load map!" << std::endl;
        return 1;
    }
    
    // Initialize UI
    std::cout << "Initializing UI..." << std::endl;
    if (!ui.initialize()) {
        std::cerr << "Failed to initialize UI!" << std::endl;
        return 2;
    }
 
    // Run UI
    std::cout << "Running simulation..." << std::endl;
    std::cout << "The simulation will display in a separate window." << std::endl;
    ui.run();
    
    // Add a timer to output simulation state periodically
    std::thread statusThread([&ui]() {
        int lastTimestep = -1;
        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            if (!ui.isRunning()) {
                std::cout << "Simulation stopped" << std::endl;
                break;
            }
            
            int currentTimestep = ui.getCurrentTimestep();
            if (currentTimestep != lastTimestep) {
                std::cout << "Current timestep: " << currentTimestep 
                          << " / " << ui.getMaxTimestep() << std::endl;
                lastTimestep = currentTimestep;
            }
            
            if (ui.isPaused()) {
                std::cout << "Simulation paused at timestep " << currentTimestep << std::endl;
                break;
            }
        }
    });
    
    statusThread.detach();  // Let the thread run independently
    
    return app.exec();
}
