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

    std::string config_file = "/home/david/webots/synthetic_data/controllers/camera_controller/config/data_generator.yaml";

    Supervisor supervisor_node(new webots::Supervisor(), TIME_STEP, config_file);

    supervisor_node.stepTime();

    std::string texture = "https://external-content.duckduckgo.com/iu/?u=https%3A%2F%2Ftse1.mm.bing.net%2Fth%3Fid%3DOIP.LjRrEK0kHLMFm-MZZhbyaQHaHa%26pid%3DApi&f=1";

    while(supervisor_node.getImageCount() < 20)
    {

    std::string object_name("TEAPOT_1");
    webots::Node* object = supervisor_node.getObject(object_name);
    double distance{1.0};
    eVector2 angles = eVector2(M_PI_4, M_PI_4/2.0);

    object->getField("texture")->setMFString(0, texture);
    supervisor_node.moveObject(object);
//    supervisor_node.moveObject(object, newposition, rotation);
    supervisor_node.focusCamera(object);
//    supervisor_node.focusCamera(object, distance, angles);
//    supervisor_node.moveCamera(newposition, distance, angles);
    supervisor_node.stepTime();

    supervisor_node.saveImages();
    }
//    supervisor_node.stepTime();
//    supervisor_node.saveImages();




//    while (supervisor_node.stepTime() != -1)
//    {

//    }




  return 0;
}
