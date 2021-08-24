#include "object.hpp"



Object::Object(webots::Node *object_node):
    object_node_(object_node)
{

//   transform_utils::fromTranslationField(object_node_->getPosition(), position_);
   transform_utils::fromRotationField(object_node_->getOrientation(), orientation_);


//   position_.x = *translations->getSFVec3f();
}

