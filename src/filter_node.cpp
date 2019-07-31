/**
 * Work in progress
 */
#include "ros/ros.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "tf/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_listener.h>
#include "LegElaboration.h"
#include <vision/Configuration.h>
#include <vision/SceneTable.h>
#include <sit_armor_injected_msgs/ArmorSITScene.h>
#include "armor_msgs/ArmorDirective.h"


using namespace ros;
using namespace tf;
using namespace ar_track_alvar_msgs;


void loadOntology(std::string path, ros::NodeHandle cn)
{

    ServiceClient client = cn.serviceClient<armor_msgs::ArmorDirective>("armor_interface_srv");
    armor_msgs::ArmorDirective srv;

    srv.request.armor_request.client_name = "";
    srv.request.armor_request.reference_name= "ontoSIT";
    srv.request.armor_request.command="LOAD";
    srv.request.armor_request.primary_command_spec="FILE";
    srv.request.armor_request.secondary_command_spec="";
    srv.request.armor_request.args.push_back( path);
    srv.request.armor_request.args.push_back( "http://www.emarolab.it/sit/simple");
    srv.request.armor_request.args.push_back( "true");
    srv.request.armor_request.args.push_back( "pellet");
    srv.request.armor_request.args.push_back( "true");

    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call service");
    }

}



void filterCallback(sit_armor_injected_msgs::SceneElementVector::Ptr msg){

    int num = msg->element.size();
    ros::NodeHandle cn;
    ServiceClient client = cn.serviceClient<sit_armor_injected_msgs::ArmorSITScene>("armor_interface_srv");
    sit_armor_injected_msgs::ArmorSITScene srv;

    srv.request.ontoReference= "ontoSIT";
    for(int i = 0; i<num; i++) {
        srv.request.sceneElements.element = msg->element;
        srv.request.learningTreshold = 0.9;
    }
    if (client.call(srv))
    {
        std::string s = srv.response.learnedSceneName;
    }
    else
    {
        ROS_ERROR("Failed to call service");
    }

}


/**
 * This is the main function where we declare the nodehandles and relevant subscribers and publishers
 * @param argc : The standard inputs
 * @param argv : The standard inputs
 * @return : A return value
 */


int main(int argc, char **argv)
{
    ros::init(argc, argv, "FilterNode");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("scene_data_4Armor", 1000000, filterCallback);

    while(n.ok()) {

        ros::spinOnce();

    }
    return 0;
}
