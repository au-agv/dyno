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

#pragma once

#include <limits>

#include <chrono/collision/bullet/ChCollisionSystemBullet.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>
#include <chrono/physics/ChSystemSMC.h>
#include <chrono_vehicle/ChWorldFrame.h>
#include <chrono_vehicle/driver/ChPathFollowerDriver.h>
#include <chrono_vehicle/utils/ChVehiclePath.h>
#include <nlohmann/json.hpp>

#include <dyno/environments/terrain.hpp>
#include <dyno/interfaces/json_configuration.hpp>
#include <dyno/models/olav.hpp>
#include <dyno/models/tracked_vehicle.hpp>
#include <dyno/serialization/hdf5_serializer.hpp>
#include <dyno/serialization/json_serializer.hpp>
#include <dyno/simulation/vehicle_failure_detector.hpp>
#include <dyno/visualization/irrlicht_tracked.hpp>
#include <dyno/visualization/irrlicht_wheeled.hpp>

#ifdef DYNO_HAS_VSG_SUPPORT
#include <dyno/visualization/vulkan_scene_graph.hpp>
#endif

namespace DYNO {
namespace Simulation {

class VehicleSimulation {
   public:
    VehicleSimulation(
        std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration);

    void Initialize();

    void Loop();

    void Step();

    void OverrideInitialPose(const chrono::ChCoordsysd& pose);

    void OverrideControlsSpeed(const double& throttle, const double& braking);

    void OverrideControlsSteering(const double& steering);

    const double& GetTime();

    const double& GetEndTime();

    const double& GetTimeStep();

    bool IsFinalTime();

    bool IsCompleted();

    std::shared_ptr<DYNO::Models::Vehicle> GetVehicle();

    std::shared_ptr<chrono::vehicle::ChDriver> GetDriver();

    std::shared_ptr<chrono::vehicle::ChTerrain> GetTerrain();

    void Dump();

    void SetOutput();

    bool ShouldStopNow();

    bool IsSuccessful() const;

    bool IsCompleted() const;

   protected:
    // Configuration
    // -------------
    std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration_;

    virtual void GetConfiguration();

    virtual void Instantiate();

    void UpdateChronoVehicleDataPath();

    void ResetChronoVehicleDataPath();

    void TeleportVehicleMotion(
        chrono::vehicle::ChWheeledVehicle* vehicle,
        const chrono::ChVector3d& pos, const chrono::ChQuaterniond& rot,
        const chrono::ChVector3d& linear_vel = chrono::ChVector3d(0, 0, 0),
        const chrono::ChVector3d& angular_vel = chrono::ChVector3d(0, 0, 0));

    // -------------

    // System
    // ------------------------------------------------------------------------
    double time_ = 0.0;
    std::shared_ptr<chrono::ChSystem> system_;
    std::shared_ptr<DYNO::Models::Vehicle> vehicle_;
    std::shared_ptr<chrono::vehicle::ChDriver> driver_;
    std::shared_ptr<DYNO::Environments::Terrain> terrain_;
    std::shared_ptr<DYNO::Visualization::Wrapper> visualization_;
    std::shared_ptr<DYNO::Serialization::Serializer> output_;
    chrono::vehicle::DriverInputs current_driver_inputs_;
    // ------------------------------------------------------------------------

    // Initialization
    // ------------------------------------------------------------------------
    void InitializeSystem();

    virtual void OverrideInitialPose();

    virtual void InitializeVehicle();

    virtual void InitializeDriver();

    virtual void InitializeSensors();

    void InitializeStraightLineDriver(const double& target_speed);

    virtual void InitializeTerrain();

    void InitializeVisualization();

    virtual void InitializeAssets();
    // ------------------------------------------------------------------------

    // Loop
    // ------------------------------------------------------------------------
    virtual void SynchronizeDriver();

    bool stop_loop_ = false;
    //  ------------------------------------------------------------------------

    // Hooks
    // ------------------------------------------------------------------------
    virtual void PreInitializeSystemHook();

    virtual void PostInitializeSystemHook();

    virtual void PostInitializeTerrainHook();

    virtual void PostInitializationHook();

    virtual void PreSynchronizationHook();

    virtual void PostSynchronizationHook();

    virtual void PostAdvanceHook();

    virtual void PostStepHook();

    virtual void WriteMetadata();

    // ------------------------------------------------------------------------

    // Simulation time
    // ------------------------------------------------------------------------

    double time_step_;

    double warmup_time_;

    double end_time_;

    bool is_completed_ = false;

    /** @brief Indicates whether or not the simulation was successful. */
    bool is_successful_ = true;

    // ------------------------------------------------------------------------

    // ------------------------------------------------------------------------

    // Driver
    // ------------------------------------------------------------------------

    void InitializeSpeedController(
        chrono::vehicle::ChSpeedController& controller);

    void InitializeSteeringController(
        chrono::vehicle::ChPathSteeringController& controller);

    bool speed_controller_overridden_ = false;

    bool steering_controller_overridden_ = false;

    // ------------------------------------------------------------------------

    // Terrain
    // ------------------------------------------------------------------------
    std::shared_ptr<chrono::ChContactMaterial> GetContactMaterial(
        const double& friction_coefficient,
        const double& restitution_coefficient = 1.0e-3,
        const double& elastic_modulus = 2.0e7) const;

    void InitializeRigidTerrain(const chrono::ChVector3d& position,
                                const double& length, const double& width,
                                const std::string& texture = "checker_black");

    void InitializeSCMTerrain(const chrono::ChVector3d& position,
                              const double& length, const double& width);
    // ------------------------------------------------------------------------

    // Output
    // ------------------------------------------------------------------------

    double output_frequency_;

    double last_output_time_ = 0.0;

    double output_trigger_position_;

    bool is_output_triggered_ = false;
    // ------------------------------------------------------------------------

    // Validation
    // ------------------------------------------------------------------------

    bool ValidateVehicleRoll(const double& threshold);

    bool ValidateVehicleYaw(const double& threshold);

    void AddGate(double gate_position_x, unsigned int position);

    std::vector<double> gates_;

    std::vector<double> gates_times_;

    unsigned int current_gate_ = 0;

    void AddTrigger(const chrono::ChVector3d& trigger_position);

    std::vector<chrono::ChVector3d> triggers_;

    std::shared_ptr<VehicleFailureDetector> failure_detector_;

   private:
    void InitializeLogger();

    void InitializeTime();

    void InitializeOutput();

    void CreateVehicle();

    void FinalizeVisualization();

    void Synchronize();

    void Advance();

    std::string log_level_;
};

}  // namespace Simulation
}  // namespace DYNO
