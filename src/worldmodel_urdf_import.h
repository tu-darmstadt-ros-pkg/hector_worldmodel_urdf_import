#ifndef WORLDMODEL_URDF_IMPORT_H
#define WORLDMODEL_URDF_IMPORT_H

#include <string>
#include <urdf/model.h>
#include <urdf_model/link.h>

namespace worldmodel_urdf_import {


class WorldModelUrdfImport
{
public:
    WorldModelUrdfImport();
    bool process(std::string param_name,
                 std::string class_id);
private:
    std::string _topic_name;
    ros::Publisher _percept_pub;

};


} //namespace worldmodel_urdf_import

#endif // WORLDMODEL_URDF_IMPORT_H
