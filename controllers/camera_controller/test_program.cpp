/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "camera_utils.hpp"
#include "supervisor_node.hpp"
#include <ctime>


#define TIME_STEP 64

// Generate random seed
std::mt19937 Supervisor::Mersenne_{
    static_cast<std::mt19937::result_type>(std::time(nullptr))};

//std::uniform_real_distribution<double> distribution1 (minVector[0],maxVector[0]);
int main() {

    eVector3 camera_z = eVector3(0.0,1.0,0.0);
    camera_z = camera_z/camera_z.norm();
    eVector3 object_y = eVector3(0.0,-1.0,.0);
    object_y = object_y/object_y.norm();
    eQuaternion q = Eigen::Quaterniond::FromTwoVectors(object_y, camera_z);
    eMatrixRot R = q.toRotationMatrix();

    double axisAngles[4];
    transform_utils::rotMatrixtoAxisAngles(R, axisAngles);
    eMatrixRot R_2;
    transform_utils::axisAnglestoRotMatrix(axisAngles, R_2);

    double angle = acos(camera_z.dot(object_y)/(camera_z.norm() * object_y.norm()))/M_PI;

    std::cout << angle << '\n';

    
  return 0;
}
