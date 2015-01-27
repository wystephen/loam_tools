#include "laser_geometry/laser_geometry.h"
//#include "laser_geometry.cpp"
#include "tf/tf.h"


#include <pcl_ros/transforms.h>
#include<pcl_ros/point_cloud.h>
#include <fstream>


ros::Subscriber sub;
ros::Publisher pub;

//serial::Serial::Serial("/dev/ttyUSB0",115200,10,eightbits,parity_none,stopbits_one,flowcontrol_none);


double w=0;
int last_sum;
double last_endtime;

//for debug
std::fstream fout;

double time_before,time_now;
double sum_before,sum_now;





int main(int argc,char **argv)
{
    ros::init(argc,argv,"sum_pointcloud");
    ros::NodeHandle n;
    ROS_INFO("Now,start the node succeful!");


    //fout.open("/home/home.txt");


//    sp.set_option(boost::asio::serial_port::baud_rate(115200));
//    sp.set_option(boost::asio::serial_port::flow_control());
//    sp.set_option(boost::asio::serial_port::parity());
//    sp.set_option(boost::asio::serial_port::stop_bits());
//    sp.set_option(boost::asio::serial_port::character_size(8));



    ROS_INFO("step2111");




    //sub=n.subscribe("first",1,lCallback);

    ros::spin();
    return 0;
}
