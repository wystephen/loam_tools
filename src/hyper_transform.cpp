//
// Created by lixin on 16-3-7.
//

#include <ros/ros.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/timer.hpp>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include  <vector>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>


#define PARAMENTER_OPTIMATION

//ros::Subscriber sub_para;
ros::Subscriber sub_laser;

ros::Publisher pub;

//Read serial data struct
boost::asio::io_service io;
boost::asio::serial_port sp(io,"/dev/ttyUSB0");

boost::mutex io_mutex;

typedef boost::shared_lock<boost::shared_mutex> readLock;
typedef boost::unique_lock<boost::shared_mutex> writeLock;

boost::shared_mutex rwmutex;


double global_angle;
char read_buf[10];


struct angle_with_time{
    double time;
    double angle;
};


angle_with_time global_angle_with_time;


//hyper_para struct
struct HyperParameter{
    int avg_v;
    double offset_r;
    double error_theta_z;

}global_para;

//device stop function
bool rotation_stop(){
    //stop  AA 55 0A 02 0C
    boost::asio::write(sp,boost::asio::buffer("\xAA",1));
    boost::asio::write(sp,boost::asio::buffer("\x55",1));
    boost::asio::write(sp,boost::asio::buffer("\xFA",1));
    boost::asio::write(sp,boost::asio::buffer("\x02",1));
    boost::asio::write(sp,boost::asio::buffer("\xFc",1));
    return true;

}

//device start function
bool rotation_start()
{
    boost::asio::write(sp,boost::asio::buffer("\xAA",1));
    boost::asio::write(sp,boost::asio::buffer("\x55",1));
    boost::asio::write(sp,boost::asio::buffer("\xFA",1));
    boost::asio::write(sp,boost::asio::buffer("\x01",1));
    boost::asio::write(sp,boost::asio::buffer("\xFB",1));
    return true;
}

//return angle only,use mutex.
double readonly(){
    readLock rdlock(rwmutex);

    return global_angle;
}
//read only function return angle with time struct data
angle_with_time readonly_time(){
    readLock rdlock(rwmutex);
    return global_angle_with_time;
}


void lCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    double angle(0.0);
    angle_with_time angle_t;
    std::cout << " angle init."<<std::endl;
    if(true)
    {

        //angle = global_angle;
        angle_t = readonly_time();

    }
    std::cout << "angle:"<<angle_t.angle<<std::endl;
    std::cout <<"[laser_time:angle_time:diff]:"<< scan_msg->header.stamp.toSec()<<":"<<angle_t.time<<":"<<angle_t.time-scan_msg->header.stamp.toSec()<<std::endl;
    if(!ros::ok()){
        rotation_stop();
    }

}


//use mutex
void writeOnly(double ang){
    writeLock wtlock(rwmutex);
    global_angle = ang;
}
void writeOnly_time(double angle,double time){
    writeLock wtlock(rwmutex);
    global_angle_with_time.angle=angle;
    global_angle_with_time.time = time;
}



void serial_process()
{
    if(!ros::ok())
    {
        rotation_stop();
        return;//close when ros node end.
    }
    while(ros::ok()) {
        memset(read_buf, 0, 10);
        if (sp.is_open()) {
            boost::asio::read(sp, boost::asio::buffer(read_buf));
        }
        double the_time = ros::Time::now().toSec();
        //std::cout <<"read buf"<<std::endl;

        for (int i = 0; i < 10; i++) {
            //std::cout << "i:"<<i<<std::endl;
            //read_buf[i] = read_buf[i] & 0xff;

            //process the
            if ((read_buf[i] & 0xff) == 0x55) {
                int a = 0xff & read_buf[i + 1];
                int b = 0xff & read_buf[i + 2];
                int c = 0xff & read_buf[i + 3];
                if ((b > -1) && (c = ((a + b) % 256)))
                {
                    int sum = a * 256 + b;
                    if (sum < 7201) {
                        if(i<5)
                            the_time -=0.01;
                       // boost::mutex::scoped_lock(io_mutex);
                        //std::cout << "sum:"<<sum << std::endl;io_lock.lock();

                        //global_angle = (double)sum / 20 / 180 * M_PI;
                        //writeOnly((double) sum /20.0/180 * M_PI);
                        writeOnly_time((double) sum /20.0/180 * M_PI,the_time);

                        //std::cout <<"global angle:"<<global_angle<<std::endl;
                    }
                }
            }
        }
    }


}

int main(int argc,char **argv)
{
    ROS_INFO("Begin Hyper transform");

    sp.set_option(boost::asio::serial_port::baud_rate(115200));
    sp.set_option(boost::asio::serial_port::flow_control());
    sp.set_option(boost::asio::serial_port::parity());
    sp.set_option(boost::asio::serial_port::stop_bits());
    sp.set_option(boost::asio::serial_port::character_size(8));

    ros::init(argc,argv,"hyper_transform");

    ros::NodeHandle node;

    pub  = node.advertise<sensor_msgs::PointCloud2>("sync_scan_cloud_filtered",1);
    sub_laser = node.subscribe("/first",1,lCallback);


    global_para.avg_v = 6000;
    global_para.offset_r = 0.04000;
    global_para.error_theta_z = 0.604000* 0.0174;

    rotation_start();

    //ros::spin();

    boost::thread serial_process_thread(&serial_process);
    serial_process_thread.detach();

    //serial_process_thread.join();
    //serial_process_thread.detach();

    while(ros::ok()){
        ros::spinOnce();
        serial_process_thread.join();
        if(!ros::ok()) rotation_stop();
    }

    return 0;
    //sub_para = node.subscribe("hyper_para",1,paraCallback);

}