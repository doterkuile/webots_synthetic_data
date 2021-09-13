#include <eigen3/Eigen/Dense>
#include <webots/Node.hpp>
#include <iostream>

// Easy to use typedef for Eigen. Used for transformation of objects

typedef Eigen::Vector2d   eVector2;
typedef Eigen::Vector3d   eVector3;
typedef Eigen::Isometry3d eMatrixHom;
typedef Eigen::Matrix3d   eMatrixRot;
typedef Eigen::Quaterniond  eQuaternion;

namespace transform_utils {

void fromField(const double *field, eVector2 &vector);
void fromField(const double *field, eVector3 &vector);
void fromField(const double *field, double &value);
void toField(const eVector2 &vector, double *field);
void toField(const eVector3 &vector, double *field);
void toField(const eMatrixHom &matrix, double *position, double *orientation);
void toField(const double value, double *field);



void toTranslationField(const eVector3 &vector, webots::Field *field);

void fromTranslationField(const webots::Field *field, eVector3 &vector);


void toRotationField(const eQuaternion &vector, webots::Field *field);

void fromRotationField(const webots::Field *field, Eigen::AngleAxisd &axis);

void fromSphericaltoCartesian(const double r, const double phi, const double theta, eVector3 &vector);

void fromQuaterniontoAxisAngles(const eQuaternion &q, double *axisAngle);

void fromRollPitchYawtoAxisAngles(const eVector3 &rpy, double *axisAngle);

void rotMatrixtoAxisAngles(const eMatrixRot &R, double *axisAngle);
void axisAnglestoRotMatrix(const double *axisAngle, eMatrixRot &R);


}

