/*
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 +                            _     _     _     _                            +
 +                           / \   / \   / \   / \                           +
 +                          ( D ) ( Y ) ( N ) ( O )                          +
 +                           \_/   \_/   \_/   \_/                           +
 +                                                                           +
 +              DYNO: Ground Vehicle Dynamics Validation Toolkit             +
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

MIT License

Copyright (c) 2024 Dario Sirangelo

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <csignal>

#include <dyno/simulation/double_lane_change.hpp>
#include <dyno/simulation/grade_climbing.hpp>
#include <dyno/simulation/sideslope_stability.hpp>
#include <dyno/simulation/sinusoidal_steering.hpp>
#include <dyno/simulation/split_surface.hpp>
#include <dyno/simulation/straight_line_acceleration.hpp>
#include <dyno/simulation/straight_line_braking.hpp>
#include <dyno/simulation/terrain_mobility.hpp>
#include <dyno/simulation/wall_to_wall.hpp>

#ifdef DYNO_HAS_SENSORS_SUPPORT
#include <dyno/simulation/autonomous_navigation.hpp>
#endif

std::shared_ptr<DYNO::Simulation::VehicleSimulation> simulation;

void HandleShutdown(int signal) {
    SPDLOG_INFO("Shutting down ...");
    simulation->Dump();
}

int main(int argc, char* argv[]) {
    // Install the signal handler.
    std::signal(SIGINT, HandleShutdown);
    std::signal(SIGTERM, HandleShutdown);
    std::signal(SIGKILL, HandleShutdown);

    // Initialize logger.
    spdlog::set_pattern("%^[%L] [DYNO@%!] > %$%v");

    // Program options
    auto configuration =
        std::make_shared<DYNO::Interfaces::JSONConfiguration>();
    configuration->Parse(argc, argv);
    configuration->ReadConfiguration();
    const auto scenario = configuration->GetScenario();

    if (scenario == "doubleLaneChange") {
        simulation =
            std::make_shared<DYNO::Simulation::DoubleLaneChange>(configuration);
    } else if (scenario == "terrainMobility") {
        simulation =
            std::make_shared<DYNO::Simulation::TerrainMobility>(configuration);
    } else if (scenario == "sideslopeStability") {
        simulation = std::make_shared<DYNO::Simulation::SideslopeStability>(
            configuration);
#ifdef DYNO_HAS_SENSORS_SUPPORT
    } else if (scenario == "autonomousNavigation") {
        simulation = std::make_shared<DYNO::Simulation::AutonomousNavigation>(
            configuration);
#endif
    } else if (scenario == "gradeClimbing") {
        simulation =
            std::make_shared<DYNO::Simulation::GradeClimbing>(configuration);

    } else if (scenario == "sinusoidalSteering") {
        simulation = std::make_shared<DYNO::Simulation::SinusoidalSteering>(
            configuration);
    } else if (scenario == "splitSurface") {
        simulation =
            std::make_shared<DYNO::Simulation::SplitSurface>(configuration);
    } else if (scenario == "straightLineAcceleration") {
        simulation =
            std::make_shared<DYNO::Simulation::StraightLineAcceleration>(
                configuration);
    } else if (scenario == "straightLineBraking") {
        simulation = std::make_shared<DYNO::Simulation::StraightLineBraking>(
            configuration);
    } else if (scenario == "wallToWallTurn") {
        simulation =
            std::make_shared<DYNO::Simulation::WallToWall>(configuration);
    } else {
        throw std::invalid_argument("Invalid scenario name.");
    }

    // Initialize the simulation.
    simulation->Initialize();

    for (unsigned int i = 0; i < (unsigned int)(simulation->GetEndTime() /
                                                simulation->GetTimeStep());
         ++i) {
        simulation->Step();

        if (simulation->ShouldStopNow()) {
            break;
        }
    }

    simulation->Dump();

    return EXIT_SUCCESS;
}
