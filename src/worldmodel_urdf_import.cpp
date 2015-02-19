#include <ros/ros.h>

#include "worldmodel_urdf_import.h"
#include <ros/console.h>
#include <hector_worldmodel_msgs/UserPercept.h>
#include <urdf_model/link.h>

namespace worldmodel_urdf_import{

WorldModelUrdfImport::WorldModelUrdfImport()
{
    ros::NodeHandle priv_nh("~");

    priv_nh.param("topic_name",  _topic_name, std::string("worldmodel/import/UserPercept"));

    ros::NodeHandle _nh;

    _percept_pub = _nh.advertise<hector_worldmodel_msgs::UserPercept>(_topic_name, 1000);

    ROS_INFO("Publishing to: %s", _topic_name.c_str());

}

bool WorldModelUrdfImport::process(std::string param_name,
                                   std::string class_id){
    urdf::Model urdf_model;
    if (!urdf_model.initParam(param_name)){
        ROS_ERROR("Failed to load urdf");
        return false;
    }

    int seq = 1;

    boost::shared_ptr<const urdf::Link> world = urdf_model.getLink("world");

    for(std::vector<boost::shared_ptr<urdf::Link> >::const_iterator it = world->child_links.begin(); it != world->child_links.end(); ++it) {
        boost::shared_ptr<urdf::Link> link = *it;

        ROS_INFO("Link: %s", link->name.c_str());
        boost::shared_ptr<urdf::Joint> joint = link->parent_joint;

        hector_worldmodel_msgs::UserPercept userPercept;

        userPercept.header.seq = seq++;

        urdf::Pose pose = joint->parent_to_joint_origin_transform;

        userPercept.pose.position.x = pose.position.x;
        userPercept.pose.position.y = pose.position.y;
        userPercept.pose.position.z = pose.position.z;

        userPercept.pose.orientation.w = pose.rotation.w;
        userPercept.pose.orientation.x = pose.rotation.x;
        userPercept.pose.orientation.y = pose.rotation.y;
        userPercept.pose.orientation.z = pose.rotation.z;


        userPercept.info.class_id = class_id;

        userPercept.info.name = link->name;
        userPercept.info.object_id = "?";

        _percept_pub.publish(userPercept);

        ROS_INFO("Percept published");


    }

    return true;
}


}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "worldmodel_urdf_import");

    ROS_INFO("worldmodel_urdf_import start");
    worldmodel_urdf_import::WorldModelUrdfImport urdf_importer;

    sleep(1);

    if(!urdf_importer.process("umad_objects_description", "dial_gauge")){
        ROS_ERROR("error on process!");
        exit(-1);
    }

    ROS_INFO("worldmodel_urdf_import done");
    exit(0);
}
