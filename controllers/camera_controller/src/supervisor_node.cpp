#include "supervisor_node.hpp"


Supervisor::Supervisor(webots::Robot *robot, int time_step):
robot_(robot),
time_step_(time_step)
{
       camera_ = robot_->getCamera("camera");
       display_ = robot_->getDisplay("segmented image display");
       camera_utils::turnCameraOn(camera_, time_step_);
       object_label_ = "Teapot";
       destination_folder_ = "/home/david/voxblox_ws/results/images/";


}

Supervisor::Supervisor(webots::Supervisor *supervisor, int time_step):
supervisor_(supervisor),
time_step_(time_step)
{
       camera_ = supervisor_->getCamera("camera");
       camera_node_ = supervisor_->getFromDef("CAMERA_1");
       display_ = supervisor_->getDisplay("segmented image display");
       camera_utils::turnCameraOn(camera_, time_step_);
       object_label_ = "Teapot";
       destination_folder_ = "/home/david/voxblox_ws/results/images/";



}

Supervisor::~Supervisor()
{
    delete supervisor_;
}

int Supervisor::stepTime()
{
    supervisor_->step(time_step_);
}

void Supervisor::saveImages()
{
    camera_utils::saveImages(camera_, display_, destination_folder_);

}

webots::Node* Supervisor::getObject(const std::string &object_name)
{
    webots::Node* object = supervisor_->getFromDef(object_name);

    return object;
}


bool Supervisor::moveObject(webots::Node* object, double translation[3], double rotation[4])
{
//        auto x = position->getMFVec3f(0);

    if(object)
    {
        object->getField("translation")->setSFVec3f(translation);
        object->getField("rotation")->setSFRotation(rotation);

       return true;

    }
    else
    {
        return false;
    }


//    position->setSFVec3f(translation);
}

void Supervisor::focusCamera(webots::Node* object)
{
    webots::Field* position = object->getField("translation");
    webots::Field* orientation = object->getField("rotation");



}

void Supervisor::moveCamera(double position[3], double distance, eVector2 angles)
{
    eVector3 new_position = eVector3(position[0], position[1], position[2]);

    camera_utils::moveCamera(camera_node_, new_position, distance,angles);
}
