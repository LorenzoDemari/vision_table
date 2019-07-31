/**
 * Work in progress
 */

#include "ros/ros.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "tf/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_listener.h>
#include <math.h>
#include "LegElaboration.h"
#include <vision/Configuration.h>
#include <vision/SceneTable.h>
#include <sit_armor_injected_msgs/SceneElement.h>
#include <sit_armor_injected_msgs/Recognition.h>
#include <sit_armor_injected_msgs/SceneElementVector.h>

#include <fstream>

# define ROWS 3
# define COLUMNS 12
# define THR 0.04
using namespace ros;
using namespace tf;
using namespace ar_track_alvar_msgs;


void init_message(vision::SceneTable::Ptr a, vision::Configuration::Ptr b, struct configuration c){

    b->leg_id = c.leg_id;
    b->name_config = c.name_config;
    b->pin=c.pin;

    a->scene.push_back( *b);

}

void init_msg_for_Armor(sit_armor_injected_msgs::SceneElement::Ptr leg, sit_armor_injected_msgs::SceneElement::Ptr pin, sit_armor_injected_msgs::SceneElementVector::Ptr msg, struct configuration c,
                        double xy[2],double p[ROWS][COLUMNS] ){

    const double x=xy[0];
    const double y=xy[1];
    leg->type=c.name_config;
    leg->features.push_back(x);
    leg->features.push_back(y);
    leg->features.push_back(0.0);

    msg->element.push_back( *leg);

    const double px = p[1][c.pin-1];
    const double py = p[2][c.pin-1];
    pin->type = "PIN_" + boost::to_string(c.pin);
    pin->features.push_back(px);
    pin->features.push_back(py);
    pin->features.push_back(0.0);

    msg->element.push_back( *pin);

}

void initialize_pins_position (double p[ROWS][COLUMNS]){

    for (int i = 0; i<12 ; i++){
        p[0][i]=i+1;
    }

    p[1][0]=-0.30;
    p[2][0]=-0.16;

    p[1][1]=-0.05;
    p[2][1]=-0.17;

    p[1][2]=0.03;
    p[2][2]=-0.15;

    p[1][3]=0.27;
    p[2][3]=-0.17;

    p[1][4]=0.24;
    p[2][4]=-0.02;

    p[1][5]=0.24;
    p[2][5]=0.05;

    p[1][6]=0.27;
    p[2][6]=0.19;

    p[1][7]=0.04;
    p[2][7]=0.17;

    p[1][8]=-0.04;
    p[2][8]=0.17;

    p[1][9]=-0.28;
    p[2][9]=0.20;

    p[1][10]=-0.24;
    p[2][10]=0.05;

    p[1][11]=-0.28;
    p[2][11]=-0.05;
}

int eval_pin (double xy [2], double p[ROWS][COLUMNS], std::string name, std::string leg){

    double x=xy[0];
    double y=xy[1];
    if (name == "NOT_X" || name == "BED_X")
        x=x-0.115;
    else if (name == "NOT_MINUS_X" || name == "BED_MINUS_X")
        x=x+0.115;
    else if (name == "NOT_Y" || name == "BED_Y")
        y=y-0.115;
    else if (name == "NOT_MINUS_Y" || name == "BED_MINUS_Y")
        y=y+0.115;
    int pin=0;
    for (int i = 0; i<COLUMNS; i++){
        if (x<p[1][i]+THR && x>p[1][i]-THR)
            if (y<p[2][i]+THR && y>p[2][i]-THR){
                pin = i+1;
                //ROS_INFO("\n\n***** %s connected to pin %d *****", leg.c_str(), pin);
                return pin;
            }
    }

    ROS_ERROR("   !!!!!!%s not connected to a PIN!!!!!!\n", leg.c_str());
    return 0;
}

void eval_config (double angles[3],tf::StampedTransform t, double xy[2], double pins[ROWS][COLUMNS], configuration &conf_leg, std::string leg_name){
    tf::Matrix3x3 m0(t.getRotation());
    m0.getEulerYPR(angles[2], angles[1], angles[0]);
    angles[0] = angles[0] * (180 / M_PI);
    angles[1] = angles[1] * (180 / M_PI);
    angles[2] = angles[2] * (180 / M_PI);
    change_angle_interval(angles);
    check_configuration(angles, conf_leg, leg_name);
    ROS_INFO("Conf leg %s:  %s", leg_name.c_str(), conf_leg.name_config.c_str());

    xy[0]=t.getOrigin().x();
    xy[1]=t.getOrigin().y();
    conf_leg.pin=eval_pin(xy, pins, conf_leg.name_config, conf_leg.leg_id);
    ROS_INFO("\n%s\n%s\nCONNECTED TO PIN %d\n", conf_leg.leg_id.c_str(), conf_leg.name_config.c_str(), conf_leg.pin);
}



/**
 * This is the main function where we declare the nodehandles and relevant subscribers and publishers
 * @param argc : The standard inputs
 * @param argv : The standard inputs
 * @return : A return value
 */


int main(int argc, char **argv)
{
    ros::init(argc, argv, "legs_node");
    ros::NodeHandle n;

    std::fstream fxy;
    fxy.open("xy.txt", std::fstream::out);
    std::fstream fypr;
    fypr.open("ypr.txt", std::fstream::out);
    std::fstream f;
    f.open("config.txt", std::fstream::out);

    //get data from tf: transforms between world and leg markers
    tf::TransformListener listener0;
    tf::TransformListener listener4;
    tf::TransformListener listener8;
    tf::TransformListener listener12;

    ros::Publisher Scene_pub = n.advertise<vision::SceneTable>("scene_data", 10);
    ros::Publisher Scene_pub_4Armor = n.advertise<sit_armor_injected_msgs::SceneElementVector>("scene_data_4Armor", 100);

    ros::Rate rate(10.0);

    double xy_100[2];
    double xy_104[2];
    double xy_108[2];
    double xy_112[2];
    double angles_100[3];
    double angles_104[3];
    double angles_108[3];
    double angles_112[3];


    double pins[ROWS][COLUMNS];
    initialize_pins_position(pins);
    struct configuration conf_leg0;
    struct configuration conf_leg4;
    struct configuration conf_leg8;
    struct configuration conf_leg12;
    int k=1;

    while(n.ok()) {


        ros::Duration(1.0).sleep();
        tf::StampedTransform transform_w_100;
        tf::StampedTransform transform_w_104;
        tf::StampedTransform transform_w_108;
        tf::StampedTransform transform_w_112;
        vision::SceneTable::Ptr ourScene (new vision::SceneTable);
        sit_armor_injected_msgs::SceneElementVector::Ptr sceneForArmor(new sit_armor_injected_msgs::SceneElementVector);


        for (int i = 0; i<3; i++){
            angles_100[i];
            angles_104[i];
            angles_108[i];
            angles_112[i];
        }
        for (int j = 0; j<2; j++){
            xy_100[j]=0;
            xy_104[j]=0;
            xy_108[j]=0;
            xy_112[j]=0;
        }

        try {
            listener0.waitForTransform("/WORLD", "/ar_marker_100", ros::Time(0), ros::Duration(0.00005));
            listener0.lookupTransform("/WORLD", "/ar_marker_100", ros::Time(0), transform_w_100);
            ROS_INFO("\n%s\n%s\nCONNECTED TO PIN %d\n", conf_leg0.leg_id.c_str(), conf_leg0.name_config.c_str(), conf_leg0.pin);
            eval_config(angles_100,transform_w_100, xy_100, pins, conf_leg0, "Leg_0");

            if (conf_leg0.pin > 0 && conf_leg0.name_config.size()>0) {
                f << conf_leg0.leg_id << std::endl << conf_leg0.name_config << std::endl << "Pin:" << conf_leg0.pin << std::endl << std::endl;
                fypr << angles_100[2] <<" " << angles_100[1] <<" " << angles_100[0] << std::endl;
                fxy << "leg 0 : " << xy_100[0] <<" " << xy_100[1] << std:: endl;
                vision::Configuration::Ptr msg0(new vision::Configuration);
                init_message(ourScene, msg0, conf_leg0);
                sit_armor_injected_msgs::SceneElement::Ptr ar0(new sit_armor_injected_msgs::SceneElement);
                sit_armor_injected_msgs::SceneElement::Ptr pin0(new sit_armor_injected_msgs::SceneElement);
                init_msg_for_Armor(ar0, pin0, sceneForArmor, conf_leg0, xy_100, pins);
            }
        }

        catch (tf::TransformException &ex100) {
            ROS_ERROR("%s", ex100.what());
        }


        try {
            listener4.waitForTransform("/WORLD", "/ar_marker_104", ros::Time(0), ros::Duration(0.00005));
            listener4.lookupTransform("/WORLD", "/ar_marker_104", ros::Time(0), transform_w_104);
            eval_config(angles_104,transform_w_104, xy_104, pins, conf_leg4, "Leg_4");

            if (conf_leg4.pin > 0 && conf_leg4.name_config.size()>0){
                f << conf_leg4.leg_id << std::endl << conf_leg4.name_config << std::endl << "Pin:" << conf_leg4.pin << std::endl << std::endl;
                fypr << angles_104[2] <<" " << angles_104[1] <<" " << angles_104[0] << std::endl;
                fxy << "leg 4: " << xy_104[0] <<" " << xy_104[1] << std:: endl;
                vision::Configuration::Ptr msg4(new vision::Configuration);
                init_message(ourScene, msg4, conf_leg4);
                sit_armor_injected_msgs::SceneElement::Ptr ar4(new sit_armor_injected_msgs::SceneElement);
                sit_armor_injected_msgs::SceneElement::Ptr pin4(new sit_armor_injected_msgs::SceneElement);
                init_msg_for_Armor(ar4, pin4, sceneForArmor, conf_leg4, xy_104, pins);
            }
        }

        catch (tf::TransformException &ex104) {
            ROS_ERROR("%s", ex104.what());
        }

        try {
            listener8.waitForTransform("/WORLD", "/ar_marker_108", ros::Time(0), ros::Duration(0.00005));
            listener8.lookupTransform("/WORLD", "/ar_marker_108", ros::Time(0), transform_w_108);
            eval_config(angles_108,transform_w_108, xy_108, pins, conf_leg8, "Leg_8");

            if (conf_leg8.pin > 0 && conf_leg8.name_config.size()>0){
                f << conf_leg8.leg_id << std::endl << conf_leg8.name_config << std::endl << "Pin:" << conf_leg8.pin << std::endl << std::endl;
                fypr << angles_108[2] <<" " << angles_108[1] <<" " << angles_108[0] << std::endl;
                fxy << "leg 8: " << xy_108[0] <<" " << xy_108[1] << std:: endl;
                vision::Configuration::Ptr msg8(new vision::Configuration);
                init_message(ourScene, msg8, conf_leg8);
                sit_armor_injected_msgs::SceneElement::Ptr ar8(new sit_armor_injected_msgs::SceneElement);
                sit_armor_injected_msgs::SceneElement::Ptr pin8(new sit_armor_injected_msgs::SceneElement);
                init_msg_for_Armor(ar8, pin8, sceneForArmor, conf_leg8, xy_108, pins);
            }

        }

        catch (tf::TransformException &ex108) {
            ROS_ERROR("%s", ex108.what());
        }

        try {
            listener12.waitForTransform("/WORLD", "/ar_marker_112", ros::Time(0), ros::Duration(0.00005));
            listener12.lookupTransform("/WORLD", "/ar_marker_112", ros::Time(0), transform_w_112);
            eval_config(angles_112,transform_w_112, xy_112, pins, conf_leg12, "Leg_12");

            if (conf_leg12.pin > 0 && conf_leg12.name_config.size()>0) {
                f << conf_leg12.leg_id << std::endl << conf_leg12.name_config << std::endl <<"Pin:" << conf_leg12.pin << std::endl << std::endl;
                fypr << angles_112[2] <<" " << angles_112[1] <<" " << angles_112[0] << std::endl;
                fxy << "leg 12: " << xy_112[0] <<" " << xy_112[1] << std:: endl;
                vision::Configuration::Ptr msg12(new vision::Configuration);
                init_message(ourScene, msg12, conf_leg12);
                sit_armor_injected_msgs::SceneElement::Ptr ar12(new sit_armor_injected_msgs::SceneElement);
                sit_armor_injected_msgs::SceneElement::Ptr pin12(new sit_armor_injected_msgs::SceneElement);
                init_msg_for_Armor(ar12, pin12, sceneForArmor, conf_leg12, xy_112, pins);
            }
        }

        catch (tf::TransformException &ex112) {
            ROS_ERROR("%s", ex112.what());
        }

        Scene_pub_4Armor.publish(sceneForArmor);
        Scene_pub.publish(ourScene);
        f << "-------------------"<< std::endl << k << std::endl;
        k++;
        ros::spinOnce();

        rate.sleep();
    }
    return 0;
}

