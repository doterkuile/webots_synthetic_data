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
time_step_(time_step),
image_count_(0)
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
    YAML::Node basenode = YAML::LoadFile(filename);
    std::get<0>(object_position_limit_) = eVector3(basenode["object_position_limit"]["min"][0].as<double>(),
                                                   basenode["object_position_limit"]["min"][1].as<double>(),
                                                   basenode["object_position_limit"]["min"][2].as<double>()
                                                   );

    std::get<1>(object_position_limit_) = eVector3(basenode["object_position_limit"]["max"][0].as<double>(),
                                                   basenode["object_position_limit"]["max"][1].as<double>(),
                                                   basenode["object_position_limit"]["max"][2].as<double>()
                                                   );
    std::get<0>(object_orientation_limit_) = eVector3(basenode["object_orientation_limit"]["min"][0].as<double>(),
                                                      basenode["object_orientation_limit"]["min"][1].as<double>(),
                                                      basenode["object_orientation_limit"]["min"][2].as<double>()
                                                      );

    std::get<1>(object_orientation_limit_) = eVector3(basenode["object_orientation_limit"]["max"][0].as<double>(),
                                                      basenode["object_orientation_limit"]["max"][1].as<double>(),
                                                      basenode["object_orientation_limit"]["max"][2].as<double>()
                                                      );

    std::get<0>(camera_distance_limit_) = basenode["camera_distance_limit"]["min"].as<double>();
    std::get<1>(camera_distance_limit_) = basenode["camera_distance_limit"]["max"].as<double>();

    std::get<0>(camera_orientation_limit_) = eVector2(basenode["camera_orientation_limit"]["min"][0].as<double>(),
                                                      basenode["camera_orientation_limit"]["min"][1].as<double>()
                                                      );

    std::get<1>(camera_orientation_limit_) = eVector2(basenode["camera_orientation_limit"]["max"][0].as<double>(),
                                                      basenode["camera_orientation_limit"]["max"][1].as<double>()
                                                      );

    synthetic_image_file_ = basenode["synthetic_image_urls"].as<std::string>();
    image_folder_ = basenode["saved_image_folder"].as<std::string>();
    parseTextureFile(synthetic_image_file_);

    return;
//    YAML::LoadFile("/home/david/webots/synthetic_data/controllers/camera_controller/config");
}


void Supervisor::parseTextureFile(const std::string &filename)
{

    std::ifstream inFile;
    std::string tex_url;

    inFile.open(filename.c_str());

    while (inFile >> tex_url)
    {
        texture_vector.push_back(tex_url);
    }


}

int Supervisor::stepTime()
{
    supervisor_->step(time_step_);
}

int Supervisor::getImageCount()
{
    return image_count_;
}

void Supervisor::saveImages()
{
    camera_utils::saveImages(camera_, display_, image_folder_, image_count_);
    image_count_++;

    return;
}

webots::Node* Supervisor::getObject(const std::string &object_name)
{
    webots::Node* object = supervisor_->getFromDef(object_name);

    return object;
}


bool Supervisor::moveObject(webots::Node* object)
{
    if(!object)
    {
        return false;
    }

    double translation[3];
    double orientation[4];
    for(int ii{0}; ii < 3; ii++)
    {
        std::uniform_real_distribution<double> distribution (std::get<0>(object_position_limit_)[ii],std::get<1>(object_position_limit_)[ii]);
        translation[ii] = distribution(Mersenne_);

    }
    eVector3 rpy;
    for(int ii{0}; ii < 3; ii++)
    {
        std::uniform_real_distribution<double> distribution (std::get<0>(object_orientation_limit_)[ii],std::get<1>(object_orientation_limit_)[ii]);
        rpy[ii] = distribution(Mersenne_);

    }
    transform_utils::fromRollPitchYawtoAxisAngles(rpy, orientation);

    this->moveObject(object, translation, orientation);

    return true;
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
    eVector2 camera_angles;
    double camera_distance;

    std::uniform_real_distribution<double> distribution1 (std::get<0>(camera_distance_limit_),std::get<1>(camera_distance_limit_));
    camera_distance = distribution1(Mersenne_);

    for(int ii{0}; ii < 2; ii++)
    {
        std::uniform_real_distribution<double> distribution2 (std::get<0>(camera_orientation_limit_)[ii],std::get<1>(camera_orientation_limit_)[ii]);
        camera_angles[ii] = distribution2(Mersenne_);

    }

    this->focusCamera(object, camera_distance, camera_angles);

    return;


}


void Supervisor::focusCamera(webots::Node* object, const double camera_distance, const eVector2 &camera_angles)
{
    webots::Field* object_position = object->getField("translation");


    eVector3 camera_position;
    transform_utils::fromSphericaltoCartesian(camera_distance, camera_angles[0], camera_angles[1], camera_position);

    eMatrixHom transform;
    transform.setIdentity();

    auto unitZ = eVector3::UnitZ();
    eVector3 unitY = eVector3::UnitY();

    eQuaternion q= Eigen::Quaterniond::FromTwoVectors(unitZ, -(camera_position));
    eMatrixRot R = q.toRotationMatrix();
    eVector3 x_camera = R * eVector3::UnitX();

    eVector3 new_x = camera_position.cross(unitY);
    double theta = acos(new_x.dot(x_camera)/(x_camera.norm() * new_x.norm()));
    eQuaternion q2 = Eigen::AngleAxisd(0.0, eVector3::UnitX())
            * Eigen::AngleAxisd(0.0, eVector3::UnitY())
            * Eigen::AngleAxisd(theta, eVector3::UnitZ());

    eVector3 position_temp;
    transform_utils::fromTranslationField(object_position, position_temp);

    transform.translate(position_temp);
    transform.translate(camera_position);

    transform.rotate(q);
    transform.rotate(q2);



    double final_position[3];
    double final_orientation[4];
    transform_utils::toField(transform, final_position,final_orientation);

    camera_node_->getField("translation")->setSFVec3f(final_position);

    camera_node_->getField("rotation")->setSFRotation(final_orientation);


    return;

}


void Supervisor::setObjectTexture(webots::Node* object)
{

}

void Supervisor::setObjectTexture(webots::Node* object, std::string &texture)
{

    object->getField("BaseColorMap")->setSFString(texture);

}



void Supervisor::moveCamera(double position[3], double distance, eVector2 angles)
{
    eVector3 new_position = eVector3(position[0], position[1], position[2]);

    camera_utils::moveCamera(camera_node_, new_position, distance,angles);
}
