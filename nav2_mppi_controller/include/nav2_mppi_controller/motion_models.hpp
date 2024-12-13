// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_
#define NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_

#include <cstdint>

#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include <xtensor/xmath.hpp>
#include <xtensor/xmasked_view.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xnoalias.hpp>

#include "nav2_mppi_controller/tools/parameters_handler.hpp"

namespace mppi
{

/**
 * @class mppi::MotionModel
 * @brief Abstract motion model for modeling a vehicle
 */
class MotionModel
{
public:
  /**
    * @brief Constructor for mppi::MotionModel
    */
  MotionModel() = default;

  /**
    * @brief Destructor for mppi::MotionModel
    */
  virtual ~MotionModel() = default;

  /**
   * @brief With input velocities, find the vehicle's output velocities
   * @param state Contains control velocities to use to populate vehicle velocities
   */
  virtual void predict(models::State & state)
  {
    using namespace xt::placeholders;  // NOLINT
    xt::noalias(xt::view(state.vx, xt::all(), xt::range(1, _))) =
      xt::view(state.cvx, xt::all(), xt::range(0, -1));

    xt::noalias(xt::view(state.wz, xt::all(), xt::range(1, _))) =
      xt::view(state.cwz, xt::all(), xt::range(0, -1));

    if (isHolonomic()) {
      xt::noalias(xt::view(state.vy, xt::all(), xt::range(1, _))) =
        xt::view(state.cvy, xt::all(), xt::range(0, -1));
    }
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  virtual bool isHolonomic() = 0;

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  virtual void applyConstraints(models::ControlSequence & /*control_sequence*/) {}
};

/**
 * @class mppi::AckermannMotionModel
 * @brief Ackermann motion model
 */
class AckermannMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::AckermannMotionModel
    */
  explicit AckermannMotionModel(ParametersHandler * param_handler, const std::string & name)
  {
    auto getParam = param_handler->getParamGetter(name + ".AckermannConstraints");
    getParam(min_turning_r_, "min_turning_r", 0.2);
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return false;
  }

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  void applyConstraints(models::ControlSequence & control_sequence) override
  {
    auto & vx = control_sequence.vx;
    auto & wz = control_sequence.wz;

    auto view = xt::masked_view(wz, (xt::fabs(vx) / xt::fabs(wz)) < min_turning_r_);
    view = xt::sign(wz) * xt::fabs(vx) / min_turning_r_;
  }

  /**
   * @brief Get minimum turning radius of ackermann drive
   * @return Minimum turning radius
   */
  float getMinTurningRadius() {return min_turning_r_;}

private:
  float min_turning_r_{0};
};

/**
 * @class mppi::DiffDriveMotionModel
 * @brief Differential drive motion model
 */
class DiffDriveMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::DiffDriveMotionModel
    */
  DiffDriveMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return false;
  }
};

/**
 * @class mppi::OmniMotionModel
 * @brief Omnidirectional motion model
 */
class OmniMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::OmniMotionModel
    */
  OmniMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return true;
  }
};

class SwerveModel : public MotionModel
{
public:
  explicit SwerveModel(ParametersHandler * param_handler, const std::string & name)
  {
    auto getParam = param_handler->getParamGetter(name + ".SwerveModelConstraints");
    getParam(min_turning_r_, "min_turning_r", 0.2);
    // getParam(vx_threshold_, "vx_threshold", 0.05);
    // getParam(vy_threshold_, "vy_threshold", 0.05);
    // getParam(wz_threshold_, "wz_threshold", 0.05);
  }

  bool isHolonomic() override { return true; }

  void applyConstraints(models::ControlSequence & control_sequence) override
  {
    auto & vx = control_sequence.vx;
    auto & vy = control_sequence.vy;
    auto & wz = control_sequence.wz;
    float zero_threshold = 0.05;
    // Modified logic:
    // Instead of simply setting wz = 0 if vy != 0, we now compare magnitudes.
    // If wz > vy, set vy = 0, else set wz = 0 (for all instances where vy != 0).
    {
      auto parallel_mask = (xt::fabs(vy) >= zero_threshold);
      auto wz_greater_mask = parallel_mask & (xt::fabs(wz) > xt::fabs(vy));
      auto vy_greater_mask = parallel_mask & (xt::fabs(wz) <= xt::fabs(vy));

      // If wz > vy, vy = 0
      xt::masked_view(vy, wz_greater_mask) = 0.0;
      // Else wz = 0
      xt::masked_view(wz, vy_greater_mask) = 0.0;
    }

    // The remainder of the logic for pure rotation / dual ackermann stays the same
    {
      auto vy_zero = (xt::fabs(vy) < zero_threshold);

      // Consider only those where wz != 0 for rotation modes
      auto rotating_mask = vy_zero & xt::not_equal(wz, 0.0);

      // Compute ratio |vx/wz|
      auto ratio = xt::eval(xt::fabs(vx) / (xt::fabs(wz) + 1e-9)); // small epsilon to avoid division by zero

      // Pure rotation mode: |vx/wz| < min_turning_r => vx=0
      auto pure_rot_mask = rotating_mask & (ratio < min_turning_r_);
      xt::masked_view(vx, pure_rot_mask) = 0.0;

      // Dual ackermann and straight line modes require no further changes here
    }
  }

private:
  float min_turning_r_{0.2};
  // float vx_threshold_{0.05};
  // float vy_threshold_{0.05};
  // float wz_threshold_{0.05};
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_
