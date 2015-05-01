#include "laser_geometry/laser_geometry.h"
#include "tf/tf.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <boost/asio.hpp>
#include <boost/timer.hpp>
#include <boost/bind.hpp>

#include <fstream>

void handle_read(char * buf,boost::system::error_code ec, std::size_t bytes_transferred);

int main(int argc, char **argv){
    ros::init(argc, argv, "publish_the_tf");

    boost::asio::io_service io;
    boost::asio::serial_port sp(io,"/dev/ttyUSB0");

    sp.set_option(boost::asio::serial_port::baud_rate(115200));
    sp.set_option(boost::asio::serial_port::flow_control());
    sp.set_option(boost::asio::serial_port::parity());
    sp.set_option(boost::asio::serial_port::stop_bits());
    sp.set_option(boost::asio::serial_port::character_size(8));

    ////////begin
    boost::asio::write(sp,boost::asio::buffer("\xAA",1));
    boost::asio::write(sp,boost::asio::buffer("\x55",1));
    boost::asio::write(sp,boost::asio::buffer("\x0A",1));
    boost::asio::write(sp,boost::asio::buffer("\x01",1));
    boost::asio::write(sp,boost::asio::buffer("\x0B",1));
    ROS_INFO("begin serial port");


    while(ros::ok()){
        char buf[5];
        boost::asio::io_service iosev;
        boost::asio::serial_port sp_tmp(iosev,"/dev/ttyUSB0");
        memset(buf,0,5);
        boost::asio::async_read(sp_tmp,boost::asio::buffer(buf),boost::bind(handle_read,buf,_1,_2));
        boost::asio::deadline_timer timer(iosev);

        timer.expires_from_now(boost::posix_time::millisec(1));
        timer.async_wait(boost::bind(&boost::asio::serial_port::cancel,boost::ref(sp_tmp)));

        iosev.run();
        sp_tmp.close();


    }

    ////////stopAA550A020C
    boost::asio::write(sp,boost::asio::buffer("\xAA",1));
    boost::asio::write(sp,boost::asio::buffer("\x55",1));
    boost::asio::write(sp,boost::asio::buffer("\x0A",1));
    boost::asio::write(sp,boost::asio::buffer("\x02",1));
    boost::asio::write(sp,boost::asio::buffer("\x0c",1));

    ros::spin();
    boost::asio::write(sp,boost::asio::buffer("\xAA",1));
    boost::asio::write(sp,boost::asio::buffer("\x55",1));
    boost::asio::write(sp,boost::asio::buffer("\x0A",1));
    boost::asio::write(sp,boost::asio::buffer("\x02",1));
    boost::asio::write(sp,boost::asio::buffer("\x0c",1));
    return 0;
}
void handle_read(char * buf,boost::system::error_code ec, std::size_t bytes_transferred){
    if(bytes_transferred > 4){
        double sum;
       // static tf::TransformBroadcaster br;
        //tf::Transform transform;
        sum = ((0xff &(*(buf+2))) * 255)+((0xff & (*(buf+3))));
        if(sum > -1 && sum <721){
            //ROS_INFO("sum is:");
            std::cout << sum << std::endl;
//            transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
//            tf::Quaternion q;
//            q.setRPY(0,0,(3.1415926 * ((sum / 4) - 90)/180));
//            transform.setRotation(q);
//            br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"/camerat","/laser"));
        }
    }
}

