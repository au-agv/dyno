#pragma once

#include <chrono/core/ChVector3.h>

namespace DYNO {
namespace Simulation {

class Obstacle {
   public:
    Obstacle(const chrono::ChVector3d& position,
             const chrono::ChVector3d& size);

    const chrono::ChVector3d& GetPosition() const;

    const chrono::ChVector3d& GetSize() const;

    const double& GetPositionX() const;

    double GetMaxSize() const;

   public:
    /**
     * @brief The x-coordinate of the obstacle's position.
     */
    chrono::ChVector3d position_;

    /**
     * @brief The y-coordinate of the obstacle's position.
     */
    chrono::ChVector3d size_;
};

}  // namespace Simulation
}  // namespace DYNO
