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
image_count_(0),
position_weight_(1.0)
{
//    this->loadWorld(world);

//       supervisor_->simulationReset();
       object_label_ = "Teapot";
       destination_folder_ = "/home/david/voxblox_ws/results/images/";
       this->parseConfig(config_file);
       this->setupCamera();







}

Supervisor::~Supervisor()
{
    delete supervisor_;
}

void Supervisor::addObject(const std::string &object_name)
{
        webots::Node* test = supervisor_->getRoot();
        webots::Field* field = test->getField("children");

        std::uniform_int_distribution<int> distribution(0,texture_vector_.size() - 1);
        int idx = distribution(Mersenne_);
        std::string texture = texture_vector_[idx];

        std::string input = "DEF " + object_name + " object_1 { translation 0.0 0 0 rotation 1 0 0 0 texture " + texture +"}";
        field->importMFNodeFromString(0, input);


}


void Supervisor::removeObject(const std::string &object_name)
{
    webots::Node* object = supervisor_->getFromDef(object_name);
    object->remove();

    return;
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
    std::get<0>(object_orientation_limit_) =  M_PI * eVector3(basenode["object_orientation_limit"]["min"][0].as<double>(),
                                                      basenode["object_orientation_limit"]["min"][1].as<double>(),
                                                      basenode["object_orientation_limit"]["min"][2].as<double>()
                                                      );

    std::get<1>(object_orientation_limit_) = M_PI * eVector3(basenode["object_orientation_limit"]["max"][0].as<double>(),
                                                      basenode["object_orientation_limit"]["max"][1].as<double>(),
                                                      basenode["object_orientation_limit"]["max"][2].as<double>()
                                                      );

    std::get<0>(camera_distance_limit_) = basenode["camera_distance_limit"]["min"].as<double>();
    std::get<1>(camera_distance_limit_) = basenode["camera_distance_limit"]["max"].as<double>();

    std::get<0>(camera_orientation_limit_) = M_PI * eVector2(basenode["camera_orientation_limit"]["min"][0].as<double>(),
                                                      basenode["camera_orientation_limit"]["min"][1].as<double>()
                                                      );

    std::get<1>(camera_orientation_limit_) = M_PI * eVector2(basenode["camera_orientation_limit"]["max"][0].as<double>(),
                                                      basenode["camera_orientation_limit"]["max"][1].as<double>()
                                                      );

    synthetic_image_file_ = basenode["synthetic_image_urls"].as<std::string>();
    lighting_file_ = basenode["lighting_file"].as<std::string>();

    texture_url_folder_ = basenode["texture_urls"].as<std::string>();
    image_folder_ = basenode["saved_image_folder"].as<std::string>();
    dataset_size_ = basenode["dataset_size"].as<int>();
    camera_size_ = basenode["camera_size"].as<int>();
    world_name_ = basenode["world_name"].as<std::string>();
    parseTextureFile(synthetic_image_file_);
    parseLightingFile(lighting_file_);


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

        std::string input_string = " PBRAppearance{ metalness 0 baseColorMap ImageTexture { url \"" +  tex_url + "\" }}";

        texture_vector_.push_back(input_string);
    }

    if (boost::filesystem::exists(texture_url_folder_))
    {
        boost::filesystem::recursive_directory_iterator iter(texture_url_folder_);
        boost::filesystem::recursive_directory_iterator end;

        while (iter != end)
        {
            std::string file_name = iter->path().string();
            std::string sub_str = ".proto";
            std::size_t pos = file_name.find(sub_str);



            if (pos !=std::string::npos)
            {

                file_name.erase(pos, sub_str.length());
                file_name.erase(0, texture_url_folder_.length());
                std::string input_string = file_name + "{}";

                auto material = file_name.find("PerforatedMetal");

                if((material != std::string::npos) || (file_name.find("WireFence") !=std::string::npos))
                {
                    // Filter out textures with holes
                    iter++;
                    continue;
                }


                texture_vector_.push_back(input_string);
            }
            iter++;
        }
//        boost::filesystem::recursive_directory_iterator iter()

    }
    else
    {
        std::cout << "Texture folder with appereances does not exist, using only web urls" << "\n";
    }



}


void Supervisor::parseLightingFile(const std::string &filename)
{
    std::ifstream inFile;
    std::string tex_url;

    inFile.open(filename.c_str());



    while (inFile >> tex_url)
    {

        std::string input_string = tex_url;

        lighting_vector_.push_back(input_string);
    }

    return;

}


void Supervisor::setLighting()
{
    if(world_name_ == "appartment")
    {
        return;
    }

    webots::Node* background = supervisor_->getFromDef("BACKGROUND_1");
    webots::Node* background_light = supervisor_->getFromDef("BG_LIGHT_1");


    std::uniform_int_distribution<int> distribution(0, lighting_vector_.size()-1);
    int idx = distribution(Mersenne_);
    std::string light_name = lighting_vector_[idx];

    webots::Field* bg_texture = background->getField("texture");
    webots::Field* bg_light_texture = background_light->getField("texture");

    bg_texture->setSFString(light_name);
    bg_light_texture->setSFString(light_name);





}


void Supervisor::loadWorld(const std::string &world_file)
{

    supervisor_->worldLoad(world_file);
    this->stepTime();

   webots::Camera* camera_ready = supervisor_->getCamera("camera");
//   supervisor_->si
   while (!camera_ready)
   {
        camera_ready = supervisor_->getCamera("camera");
   }

   return;


}

void Supervisor::reloadWorld(const std::string &world_file)
{
    supervisor_->worldReload();
//    delete supervisor_;
//    supervisor_ = new webots::Supervisor();
    this->stepTime();
    return;


}


void Supervisor::setBasePosition(std::string &world)
{
    std::vector<eVector3> positions;
    if(world == "test")
    {
        positions.push_back(eVector3(0, 0.15, 0));
        position_weight_ = 1.0;
    }
    else if(world == "appartment")
    {

        positions.push_back(eVector3(2, 0.85, 10.2));
        positions.push_back(eVector3(2.6, 0.85, 6));
        positions.push_back(eVector3(3.3, 0.85, 1.5));
        position_weight_ = 2.0;


    }
    else if(world == "village")
    {

        positions.push_back(eVector3(20, 0 , -160));
        positions.push_back(eVector3(540, 0, -200));
        position_weight_ = 2.0;

    }

    std::uniform_int_distribution<int> distribution(0,positions.size() - 1 );
    int idx = distribution(Mersenne_);
    object_base_position_ = positions[idx];


}





void Supervisor::setupCamera()
{
    camera_ = supervisor_->getCamera("camera");
    camera_node_ = supervisor_->getFromDef("CAMERA_1");
    display_ = supervisor_->getDisplay("segmented image display");
//    supervisor_->setCustomData();
//    camera_node_->getField("width")->setSFInt32(camera_size_);
//    camera_node_->getField("height")->setSFInt32(camera_size_);
//    std::cout << camera_->getHeight() << "\n";
//    std::cout << camera_->getHeight() << "\n";

    camera_utils::turnCameraOn(camera_, time_step_);
}

int Supervisor::stepTime()
{
    supervisor_->step(time_step_);
}

bool Supervisor::checkImageCount()
{
    return image_count_ < dataset_size_;
}


bool Supervisor::checkBottomVisibility(webots::Node* object)
{
    webots::Field* camera_orientation = this->camera_node_->getField("rotation");
    webots::Field* object_orientation = object->getField("rotation");


    eMatrixRot R_c, R_o;

    const double *rot_c = {camera_orientation->getSFRotation()};
    const double *rot_o = {object_orientation->getSFRotation()};

    transform_utils::axisAnglestoRotMatrix(rot_c, R_c);
    transform_utils::axisAnglestoRotMatrix(rot_o, R_o);

    eVector3 e_z_c = R_c * eVector3::UnitZ();
    eVector3 e_y_o = R_o * eVector3::UnitY();

    double angle = acos(e_z_c.dot(e_y_o)/(e_z_c.norm() * e_y_o.norm()))/M_PI;

    if(angle > 0.5)
    {
        return true;
    }
    else{
        return false;
    }

}


bool Supervisor::saveImages(webots::Node* object)
{
    if(!this->checkBottomVisibility(object))
    {
        return false;
    }

    if(camera_utils::saveImages(camera_, display_,  object, image_folder_, image_count_))
    {
        image_count_++;
        return true;

    }

    return false;
}

webots::Node* Supervisor::getObject(const std::string &object_name)
{
    webots::Node* object = supervisor_->getFromDef(object_name);

    return object;
}

webots::Node* Supervisor::getAppearance(webots::Node* object)
{
    webots::Field* appearance = object->getField("texture");

    webots::Node* root = supervisor_->getRoot();

//    appearance->importSFNode("/usr/local/webots/projects/appearances/protos/Asphalt.proto");
    appearance->importSFNodeFromString("OldPlywood{}");
    this->stepTime();
//    appearance->

    return object;
}


bool Supervisor::moveObject(webots::Node* object)
{
    if(!object)
    {
        return false;
    }

    setBasePosition(world_name_);

    double translation[3];
    double orientation[4];
    for(int ii{0}; ii < 3; ii++)
    {
        std::uniform_real_distribution<double> distribution (std::get<0>(object_position_limit_)[ii],std::get<1>(object_position_limit_)[ii]);
        translation[ii] = object_base_position_[ii] +  position_weight_ * distribution(Mersenne_);

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


bool Supervisor::smallObjectDisplacement(webots::Node *object)
{

    if(!object)
    {
        return false;
    }
    webots::Field* object_position = object->getField("translation");

    double translation[3];
    eVector3 test;
    transform_utils::fromTranslationField(object_position, test);
    for(int ii{0}; ii < 3; ii++)
    {
         //        Make sure heigth of the object gets not altered
        if(ii == 1)
        {
            translation[ii] = test[ii];
            continue;
        }
        std::uniform_real_distribution<double> distribution (-0.3, 0.3);
        translation[ii] = test[ii] + camera_distance_* distribution(Mersenne_);

    }

    const double *orientation = object->getField("rotation")->getSFRotation();
    this->moveObject(object, translation, orientation);


    return true;
}


bool Supervisor::moveObject(webots::Node* object, const double translation[3], const double rotation[4])
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
//    double camera_distance;

    std::uniform_real_distribution<double> distribution1 (std::get<0>(camera_distance_limit_),std::get<1>(camera_distance_limit_));
    camera_distance_ = position_weight_ * distribution1(Mersenne_);

    for(int ii{0}; ii < 2; ii++)
    {
        std::uniform_real_distribution<double> distribution2 (std::get<0>(camera_orientation_limit_)[ii],std::get<1>(camera_orientation_limit_)[ii]);
        camera_angles[ii] = distribution2(Mersenne_);

    }

    this->focusCamera(object, camera_distance_, camera_angles);

    return;


}



void Supervisor::focusCamera(webots::Node* object, const double camera_distance, const eVector2 &camera_angles)
{
    webots::Field* object_position = object->getField("translation");


    eVector3 camera_position;
    transform_utils::fromSphericaltoCartesian(camera_distance, camera_angles[0], camera_angles[1], camera_position);

    eMatrixHom transform;
    eMatrixHom transform2;
    transform.setIdentity();
    transform2.setIdentity();

    auto unitZ = eVector3::UnitZ();
    eVector3 unitY = eVector3::UnitY();

    eQuaternion q= Eigen::Quaterniond::FromTwoVectors(unitZ, -(camera_position));
    eMatrixRot R = q.toRotationMatrix();
    eVector3 x_camera = R * eVector3::UnitX();

    eVector3 new_x = camera_position.cross(unitY);
    double costheta = new_x.dot(x_camera)/(x_camera.norm() * new_x.norm());

    double theta = acos(new_x.dot(x_camera)/(x_camera.norm() * new_x.norm()));




    eVector3 position_temp;
    transform_utils::fromTranslationField(object_position, position_temp);

    transform.translate(position_temp);
    transform.translate(camera_position);

    transform2.translate(position_temp);
    transform2.translate(camera_position);

    transform.rotate(q);
    transform2.rotate(q);

    eVector3 test_y1 = transform.rotation() * eVector3::UnitY();
    if(test_y1[0] > 0.0)
    {
        theta = -theta;
    }

    eQuaternion q2 = Eigen::AngleAxisd(0.0, eVector3::UnitX())
                   * Eigen::AngleAxisd(0.0, eVector3::UnitY())
                   * Eigen::AngleAxisd(theta, eVector3::UnitZ());


    eQuaternion q3 = Eigen::AngleAxisd(0.0, eVector3::UnitX())
                   * Eigen::AngleAxisd(0.0, eVector3::UnitY())
                   * Eigen::AngleAxisd(-theta, eVector3::UnitZ());


    transform.rotate(q2);
    transform2.rotate(q3);

    eVector3 test_y2 = transform.rotation() * eVector3::UnitY();

    eVector3 test_y3 = transform2.rotation() * eVector3::UnitY();


    double final_position[3];
    double final_orientation[4];
    eVector3 camera_pos = transform.translation();

    transform_utils::toField(transform, final_position,final_orientation);

    camera_node_->getField("translation")->setSFVec3f(final_position);

    camera_node_->getField("rotation")->setSFRotation(final_orientation);


    return;

}


void Supervisor::setObjectTexture(webots::Node* object)
{
    std::uniform_int_distribution<int> distribution(0,texture_vector_.size()-1);
    int idx = distribution(Mersenne_);
    std::string texture = texture_vector_[idx];

    this->setObjectTexture(object, texture);
}

void Supervisor::setObjectTexture(webots::Node* object, std::string &texture)
{

   webots::Field* appearance = object->getField("texture");
//   webots::Field* appearance2 = object->getProtoField("texture");
   webots::Node* node = appearance->getSFNode();
   appearance->getTypeName();

   if(node)
   {
       appearance->removeSF();
   }

   appearance->importSFNodeFromString(texture);
//   std::cout << node->getBaseTypeName() << '\n';

//   webots::Node* node = appearance->getSFNode();

//   node->remove();
//   supervisor_->
   this->stepTime();
//   appearance = object->getField("texture");
//   appearance->importSFNodeFromString(texture);


}



void Supervisor::moveCamera(double position[3], double distance, eVector2 angles)
{
    eVector3 new_position = eVector3(position[0], position[1], position[2]);

    camera_utils::moveCamera(camera_node_, new_position, distance,angles);
}
