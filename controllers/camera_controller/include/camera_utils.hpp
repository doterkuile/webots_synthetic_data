#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <memory>
#include <iostream>
#include "transform_utils.hpp"

namespace camera_utils{


void turnCameraOn(webots::Camera* camera, int time_step);


void saveImages(webots::Camera* camera, webots::Display* display, std::string destinationFolder);


std::vector<webots::CameraRecognitionObject> getObjectList(webots::Camera *camera);

void moveCamera(webots::Node *camera_node, const eVector3 &objectPosition, const double distance, const eVector2 &angles);









} //Closing namespace