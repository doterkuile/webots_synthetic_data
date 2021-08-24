#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include "camera_utils.hpp"
#include "transform_utils.hpp"
#include "object.hpp"
#include <random>
#include <yaml-cpp/yaml.h>


class Supervisor
{
public:
    Supervisor(webots::Robot* robot,int time_step);
    Supervisor(webots::Supervisor* supervisor,int time_step, const std::string &config_file);


    ~Supervisor();

    void saveImages();
    int stepTime();
    webots::Node* getObject(const std::string &object_name);
    bool moveObject(webots::Node* object, double translation[3], double rotation[4]);
    void focusCamera(webots::Node* object);
    void moveCamera(double position[3], double distance, eVector2 angles);
    void parseConfig(const std::string &filename);

private:

    // Initialize meresse twister with random seed based on the clock:
    static std::mt19937 Mersenne_;
    std::tuple<eVector3, eVector3> object_position_limit_;
    std::tuple<eVector3, eVector3> object_orientation_limit_;
    std::tuple<double, double> camera_distance_limit_;
    std::tuple<eVector2, eVector2> camera_orientation_limit_;
    webots::Supervisor* supervisor_;
    webots::Robot* robot_;
    webots::Camera* camera_;
    webots::Node* camera_node_;
    webots::Display* display_;
    int time_step_;
    std::string destination_folder_;
    std::vector<webots::CameraRecognitionObject> object_list_;
    std::string object_label_;



};
