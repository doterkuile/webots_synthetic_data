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

/*
 * Description:  An example of use of a camera device with recognition segmentation capability.
 */

#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <memory>
#include <iostream>


#define SPEED 1.5
#define TIME_STEP 64

void turnOnCamera(webots::Camera* camera, int time_step)
{
    camera->enable(TIME_STEP);
    camera->recognitionEnable(TIME_STEP);
    camera->enableRecognitionSegmentation();
}

void saveImages(webots::Camera* camera, webots::Display* display, std::string destinationFolder)
{
    if(camera->isRecognitionSegmentationEnabled() && camera->hasRecognition() && camera->getRecognitionSamplingPeriod() > 0 )
    {
        std::string filenamesegment = destinationFolder + "testsegmentationimage.jpg";
        std::string filename = destinationFolder + "testimage.jpg";

        auto image1 = camera->getRecognitionSegmentationImage();
        if( image1)
        {
            auto save1 = camera->saveRecognitionSegmentationImage(filenamesegment, 100);
            auto segmentedImage = display->imageNew(camera->getWidth(),camera->getHeight(), image1, webots::Display::BGRA);
            display->imagePaste(segmentedImage, 0, 0 , false);
            display->imageDelete(segmentedImage);
        }


        auto image2 = camera->getImage();
        auto save2 =camera->saveImage(filename, 100);
    }

}

std::vector<webots::CameraRecognitionObject> getObjectList(webots::Camera* camera)
{
    int nrOfObjects = camera->getRecognitionNumberOfObjects();
    std::vector<webots::CameraRecognitionObject> objectList(nrOfObjects);

    const webots::CameraRecognitionObject* objectIt = camera->getRecognitionObjects();
    for(auto it = objectList.begin(); it != objectList.end(); it++)
    {
        *it = *objectIt;
        objectIt++;
    }

    return objectList;

}




int main() {

    webots::Robot *robot = new webots::Robot();

    webots::Camera* camera = robot->getCamera("camera");
    webots::Display* display = robot->getDisplay("segmented image display");
    turnOnCamera(camera, TIME_STEP);

    std::string destinationFolder = "/home/david/voxblox_ws/results/images/";
    std::vector<webots::CameraRecognitionObject> objectList;

    robot->step(TIME_STEP);
    saveImages(camera, display, destinationFolder);
    objectList = getObjectList(camera);

    delete robot;


  return 0;
}
