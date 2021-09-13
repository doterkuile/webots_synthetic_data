#include "camera_utils.hpp"

namespace camera_utils{

void turnCameraOn(webots::Camera* camera, int time_step)
{
    camera->enable(time_step);
    camera->recognitionEnable(time_step);
    camera->enableRecognitionSegmentation();
}

bool saveImages(webots::Camera* camera, webots::Display* display, webots::Node* object, std::string destinationFolder, int image_count)
{
    if(camera->isRecognitionSegmentationEnabled() && camera->hasRecognition() && camera->getRecognitionSamplingPeriod() > 0 )
    {
        std::string filenamesegment = destinationFolder + "segmentationimage_" + std::to_string(image_count) + ".jpg";
        std::string filename = destinationFolder + "image_" + std::to_string(image_count) + ".jpg";


        auto image1 = camera->getRecognitionSegmentationImage();
        if( image1)
        {
//            auto save1 = camera->saveRecognitionSegmentationImage(filenamesegment, 100);
            auto segmentedImage = display->imageNew(camera->getWidth(),camera->getHeight(), image1, webots::Display::BGRA);
            display->imagePaste(segmentedImage, 0, 0 , false);
//            display->imageDelete(segmentedImage);
        }
        int nr_of_objects = camera->getRecognitionNumberOfObjects();
        const webots::CameraRecognitionObject* objects = camera->getRecognitionObjects();
        const webots::CameraRecognitionObject* end = objects + nr_of_objects;

        for(auto it = objects; it != end; ++it)
        {
            if( (std::string(it->model) == "object_1"))
            {
                auto image2 = camera->getImage();
                auto save2 =camera->saveImage(filename, 100);

                return true;
            }
        }

        return false;


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

void moveCamera(webots::Node *camera_node, const eVector3 &objectPosition, const double distance, const eVector2 &angles)
{
    eVector3 newPosition;
    transform_utils::fromSphericaltoCartesian(distance, angles.x(), angles.y(), newPosition);
    eMatrixHom transform;
    transform.setIdentity();


//    q = eQuaternion.FromTwoVectors(newPosition - objectPosition, newPosition);

    eVector3 unitZ{0,0,1};
    eVector3 unitY{0,1,0};

    eQuaternion q = Eigen::Quaterniond::FromTwoVectors(unitZ, -(newPosition));

    transform.translate(objectPosition);

    transform.translate(newPosition);
//    transform.rotate(q);
    eMatrixRot R = q.toRotationMatrix();

    eVector3 y_new = R * unitY;

    eQuaternion q2 = Eigen::Quaterniond::FromTwoVectors(unitY, y_new);
//    transform.rotate(q2);
//    eQuaternion q3 = transform.rotation().




    double newvector[3];
    for(int ii{0}; ii < 3; ii++)
    {
        eVector3 x = transform.translation();
        newvector[ii] = x[ii];
    }


    double newrotation[4];
    transform_utils::rotMatrixtoAxisAngles(transform.rotation(), newrotation);


    camera_node->getField("translation")->setSFVec3f(newvector);
    camera_node->getField("rotation")->setSFRotation(newrotation);


}






} //Closing namespace
