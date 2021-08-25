#include "transform_utils.hpp"


namespace transform_utils {


void toField(const double value, double *field)
{
    *field = value;
}

void toField(const eVector2 &vector, double *field)
{
    for(int ii{0}; ii < vector.size(); ii++)
    {
    }
}
void toField(const eVector3 &vector, double *field)
{
    for(int ii{0}; ii < vector.size(); ii++)
    {
        field[ii] = vector[ii];
    }
}

void toField(const eMatrixHom &matrix, double *position, double *orientation)
{
    eVector3 t = matrix.translation();
    eMatrixRot R = matrix.rotation();
    toField(t, position);
    rotMatrixtoAxisAngles(R, orientation);
}



void fromTranslationField(const webots::Field *field, eVector3 &vector)
{
    for(int ii{0}; ii <vector.size(); ii++)
    {
        vector[ii] = field->getSFVec3f()[ii];
    }
}


void fromRotationField(const double *field, eQuaternion &vector)
{
;
    for(int ii{0}; ii <3; ii++)
    {
//        vector[ii] = field->getSFVec3f()[ii];

    }
}






void fromSphericaltoCartesian(const double r, const double phi, const double theta, eVector3 &vector)
{

    vector.x() = r * sin(phi) * cos(theta);
    vector.y() = r * sin(phi) * sin(theta);
    vector.z() = r * cos(phi);

    return;
}

void fromQuaterniontoAxisAngles(const eQuaternion &q, double *axisAngle)
{

    axisAngle[0] = q.x()/std::sqrt(1-q.w() * q.w());
    axisAngle[1] = q.y()/std::sqrt(1-q.w() * q.w());
    axisAngle[2] = q.z()/std::sqrt(1-q.w() * q.w());
    axisAngle[3] = 2 * std::acos(q.w());


    return;
}

void fromRollPitchYawtoAxisAngles(const eVector3 &rpy, double *axisAngle)
{

    eQuaternion q2 = Eigen::AngleAxisd(rpy[0], eVector3::UnitX())
                   * Eigen::AngleAxisd(rpy[1], eVector3::UnitY())
                   * Eigen::AngleAxisd(rpy[2], eVector3::UnitZ());

    fromQuaterniontoAxisAngles(q2, axisAngle);

    return;

}

void rotMatrixtoAxisAngles(const eMatrixRot &R, double *axisAngle)
{
    Eigen::AngleAxisd X(R);
    axisAngle[0] = X.axis()[0];
    axisAngle[1] = X.axis()[1];
    axisAngle[2] = X.axis()[2];

    axisAngle[3] = X.angle();

    return;
}

} // close namespace
