#include "laser_geometry/laser_geometry.h"
#include "tf/tf.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <boost/asio.hpp>

int main(int argc, char **argv){
    ros::init(argc, argv, "only_tf");


    static tf::TransformBroadcaster br;
    tf::Transform transform;


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

    char bufread[6];
    int a(0),b(0),c(0),sum(0);
    char readit[1];
    double w(0.0);
    double size_of_read = 6;
    double last_time = ros::Time::now().toSec();
    while(ros::ok()){
        memset(bufread,0,size_of_read);
        if(sp.is_open()){
            boost::asio::read(sp,boost::asio::buffer(bufread));

        }else{
            ROS_INFO("sp is not open");
        }
        for (int i(0); i< size_of_read; i++){
            bufread[i] = bufread[i] & 0xff;

        }
        for(int i = 0;i<size_of_read-4;i++){
            if((bufread[i] & 0xff) == 0x55)
            {
                a = 0xff & bufread[i+1];
                b = 0xff & bufread[i+2];
                c = 0xff & bufread[i+3];
                if((b>-1) && (c == (a+b) % 255))
                {
                    if(c != (a+b))
                    {
                        ROS_INFO("c error");
                        continue;
                    }
                    sum = a * 255 + b;
                    if((sum < 0) || (sum > 720)){
                        ROS_INFO("sum error");
                        continue;
                    }
                }

            }
        }
        //        while(1){
        //            boost::asio::read(sp,boost::asio::buffer(readit));
        //            if( (readit[0] & 0xff) == 0xaa){
        //                char read_tmp[4];
        //                boost::asio::read(sp,boost::asio::buffer(read_tmp));

        //                a = 0xff & bufread[1];
        //                b = 0xff & bufread[2];
        //                c = 0xff & bufread[3];
        //                if((b>-1) && (c == (a+b) % 255)){
        //                    if(c != (a+b))
        //                    {
        //                        ROS_INFO("c error");
        //                        continue;
        //                    }
        //                    sum = a * 256 + b;
        //                    if((sum < 0) || (sum > 720)){
        //                        ROS_INFO("sum error");
        //                        continue;
        //                    }
        //                }
        //                break;
        //            }
        //        }

        transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
        tf::Quaternion q;
        q.setRPY(0,0,(3.1415926 * ((sum / 4) - 90)/180));
        transform.setRotation(q);
        std::cout << ros::Time::now().toSec() - last_time << std::endl;
        last_time = ros::Time::now().toSec();
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/camera_t","/laser"));



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


