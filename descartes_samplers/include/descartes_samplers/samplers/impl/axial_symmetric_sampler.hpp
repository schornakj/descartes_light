/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef DESCARTES_SAMPLERS_SAMPLERS_IMPL_AXIAL_SYMMETRIC_SAMPLER_HPP
#define DESCARTES_SAMPLERS_SAMPLERS_IMPL_AXIAL_SYMMETRIC_SAMPLER_HPP

#include "descartes_samplers/samplers/axial_symmetric_sampler.h"

const static std::size_t opw_dof = 6;

namespace descartes_light
{

template<typename FloatType>
AxialSymmetricSampler<FloatType>::AxialSymmetricSampler(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& tool_pose,
                                                        const typename KinematicsInterface<FloatType>::Ptr robot_kin,
                                                        const FloatType radial_sample_resolution,
                                                        const typename CollisionInterface<FloatType>::Ptr collision)
  : tool_pose_(tool_pose)
  , kin_(robot_kin)
  , collision_(collision)
  , radial_sample_res_(radial_sample_resolution)
{

}

template<typename FloatType>
bool AxialSymmetricSampler<FloatType>::isCollisionFree(const FloatType* vertex)
{
  return collision_->validate(vertex, opw_dof);
}

template<typename FloatType>
bool AxialSymmetricSampler<FloatType>::sample(std::vector<FloatType>& solution_set)
{
  std::vector<FloatType> buffer;

  const auto nSamplesInBuffer = [] (const std::vector<FloatType>& v) -> std::size_t {
    return v.size() / opw_dof;
  };

  FloatType angle = static_cast<FloatType>(-1.0 * M_PI);

  while (angle <= static_cast<FloatType>(M_PI)) // loop over each waypoint
  {
    Eigen::Transform<FloatType, 3, Eigen::Isometry> p = tool_pose_ * Eigen::AngleAxis<FloatType>(angle, Eigen::Matrix<FloatType, 3, 1>::UnitZ());
    kin_->ik(p, buffer);

    const auto n_sols = nSamplesInBuffer(buffer);
    for (std::size_t i = 0; i < n_sols; ++i)
    {
      const auto* sol_data = buffer.data() + i * opw_dof;
      if (AxialSymmetricSampler<FloatType>::isCollisionFree(sol_data))
        solution_set.insert(end(solution_set), sol_data, sol_data + opw_dof);
    }
    buffer.clear();

    angle += radial_sample_res_;
  } // redundancy resolution loop

  return !solution_set.empty();
}

} // namespace descartes_light

#endif // DESCARTES_SAMPLERS_SAMPLERS_IMPL_AXIAL_SYMMETRIC_SAMPLER_HPP
