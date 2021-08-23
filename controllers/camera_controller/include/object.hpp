#include <string>
#include <iostream>
#include <webots/Node.hpp>
#include "transform_utils.hpp"



class Object{


public:

    Object(webots::Node *object_node);


//    ~Object();

    void translate(const double translation[3]);

    void translate(const eVector3 &translation);

    void rotate(const eQuaternion &rotation);

    void transform(const eMatrixHom &transform);

    void setColor(const eVector3 &color);

    void setAppearance();

    eVector3 getPosition() const;
    eQuaternion getOrientation() const;
    eMatrixHom getTransform() const;
    std::string getLabel() const;
    eVector3 getColor() const;
    std::string getAppearance() const;




private:
    webots::Node* object_node_;

    eVector3 position_;
    eQuaternion orientation_;
    std::string label_;
    eVector3 color_;
    std::string appearance_;




};
