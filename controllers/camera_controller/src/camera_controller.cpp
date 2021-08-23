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


#define TIME_STEP 64


int main() {

    Supervisor supervisor_node(new webots::Supervisor(), TIME_STEP);

    supervisor_node.stepTime();

//    supervisor_node.saveImages();
    double newposition[3]  = {0.5, 0.0, 0.0};
    double rotation[4]     = {1.0, 0.0, 0.0, 0.0};
//    supervisor_node.saveImages();
    std::string object_name("TEAPOT_1");
    webots::Node* object = supervisor_node.getObject(object_name);
    double distance{1.0};
    eVector2 angles = eVector2(M_PI_4, M_PI_4/2.0);

    supervisor_node.moveObject(object, newposition, rotation);
    supervisor_node.moveCamera(newposition, distance, angles);
    supervisor_node.stepTime();

    supervisor_node.saveImages();
    supervisor_node.stepTime();
    supervisor_node.saveImages();




//    while (supervisor_node.stepTime() != -1)
//    {

//    }




  return 0;
}