#include <ros/ros.h>

#include "worldmodel_urdf_import.h"
#include <ros/console.h>
#include <hector_worldmodel_msgs/UserPercept.h>
#include <urdf_model/link.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <XmlRpcException.h>

namespace worldmodel_urdf_import{

WorldModelUrdfImport::WorldModelUrdfImport()
{
    ros::NodeHandle priv_nh("~");

    priv_nh.param("topic_name",  _topic_name, std::string("worldmodel/user_percept"));

    _percept_pub = _nh.advertise<hector_worldmodel_msgs::UserPercept>(_topic_name, 1000);

    ROS_INFO("Publishing to: %s", _topic_name.c_str());

}

void WorldModelUrdfImport::processConfig(){

    ros::NodeHandle priv_nh("~");

    XmlRpc::XmlRpcValue urdf_import_names;
    if (priv_nh.getParam("imports", urdf_import_names) && urdf_import_names.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for(int i = 0; i < urdf_import_names.size(); ++i) {
            XmlRpc::XmlRpcValue item = urdf_import_names[i];
            if (!item.hasMember("urdf_name")) {
                ROS_ERROR("Urdf %d could not be imported: no urdf name given", i);
                continue;
            }

            std::string urdf_name = item["urdf_name"];

            if (!item.hasMember("class_id")) {
                ROS_ERROR("Urdf %d could not be imported: no class_id given", i);
                continue;
            }
            std::string class_id = item["class_id"];

            if (!item.hasMember("frame_id")) {
                ROS_ERROR("Urdf %d could not be imported: no frame_id given", i);
                continue;
            }
            std::string frame_id = item["frame_id"];

            ROS_INFO("Importing urdf %d", i);
            this->process(urdf_name, class_id, frame_id);
        }
    }

}

bool WorldModelUrdfImport::process(std::string param_name,
                                   std::string class_id, std::string frame_id){
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

        userPercept.header.frame_id = frame_id;
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

        userPercept.info.object_id = link->name;

        userPercept.info.class_support = 1;
        userPercept.info.object_support = 1;


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

    sleep(2);

    urdf_importer.processConfig();

    ROS_INFO("worldmodel_urdf_import done");
    exit(0);
}
