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

Supervisor::Supervisor(webots::Supervisor *supervisor, int time_step, const std::string &config_file):
supervisor_(supervisor),
time_step_(time_step)
{
       camera_ = supervisor_->getCamera("camera");
       camera_node_ = supervisor_->getFromDef("CAMERA_1");
       display_ = supervisor_->getDisplay("segmented image display");
       camera_utils::turnCameraOn(camera_, time_step_);
       object_label_ = "Teapot";
       destination_folder_ = "/home/david/voxblox_ws/results/images/";
       this->parseConfig(config_file);



}

Supervisor::~Supervisor()
{
    delete supervisor_;
}

void Supervisor::parseConfig(const std::string &filename)
{
//    YAML::Node basenode = YAML::LoadFile(filename);
////    std::get<0>(object_position_limit_)[0]
//            auto test = basenode["object_position_limit"]["min"][0].as<double>();
    return;
//    YAML::LoadFile("/home/david/webots/synthetic_data/controllers/camera_controller/config");
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
    webots::Field* object_position = object->getField("translation");
    webots::Field* orientation = object->getField("rotation");


    double distance{1.0};
    eVector2 angles = eVector2(M_PI_4, M_PI_4* 0.1);

    eVector3 camera_position;
    transform_utils::fromSphericaltoCartesian(distance, angles[0], angles[1], camera_position);

    eMatrixHom transform;
    transform.setIdentity();

    auto unitZ = eVector3::UnitZ();
    eVector3 unitY = eVector3::UnitY();

    eQuaternion q= Eigen::Quaterniond::FromTwoVectors(unitZ, -(camera_position));
    eMatrixRot R = q.toRotationMatrix();
    eVector3 x_camera = R * eVector3::UnitX();
    eVector3 z_camera = R * unitZ;
    auto euler_angles = R.eulerAngles(0,1,2);

    eVector3 new_x = camera_position.cross(unitY);
    double theta = acos(new_x.dot(x_camera)/(x_camera.norm() * new_x.norm()));
    eQuaternion q2 = Eigen::Quaterniond::FromTwoVectors(new_x, x_camera);
    eQuaternion q3 = Eigen::Quaterniond::FromTwoVectors(x_camera, new_x);

    auto R2 = q2.toRotationMatrix().eulerAngles(0,1,2);
////    R.eul
    q2 = Eigen::AngleAxisd(0.0, eVector3::UnitX())
            * Eigen::AngleAxisd(0.0, eVector3::UnitY())
            * Eigen::AngleAxisd(theta, eVector3::UnitZ());
//    transform.translate(object_position);
    eVector3 position_temp;
    transform_utils::fromTranslationField(object_position, position_temp);
//    camera_position[1] = 0;
    transform.translate(position_temp);
    transform.translate(camera_position);
//    transform.translate(R);
    transform.rotate(q);
    this->stepTime();
    transform.rotate(q2);
    this->stepTime();

//    transform_utils.


    double final_position[3];
    double final_orientation[4];
    transform_utils::toField(transform, final_position,final_orientation);

    camera_node_->getField("translation")->setSFVec3f(final_position);
//    final_orientation[3] = 0;
    camera_node_->getField("rotation")->setSFRotation(final_orientation);


    return;

}

void Supervisor::moveCamera(double position[3], double distance, eVector2 angles)
{
    eVector3 new_position = eVector3(position[0], position[1], position[2]);

    camera_utils::moveCamera(camera_node_, new_position, distance,angles);
}
