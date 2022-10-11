#include "imageBasics.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "extract_image");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::Subscriber subpc = n.subscribe("/livox/lidar", 10, pc2Callback);
    ros::Subscriber subimg = n.subscribe("/zed2/zed_node/left/image_rect_color", 1, imgCallback);
    //  ros::Subscriber subpos = n.subscribe("/mavros/local_position/pose", 1000, poseCallback);
    //  ros::Subscriber subvel = n.subscribe("/mavros/local_position/velocity_body", 1000, velCallback);

    //  ros::Subscriber subdetection = n.subscribe("/darknet_ros/bounding_boxes", 1000, detectCallback);  //dependency: darknet_ros_msgs

    pubimg = n.advertise<sensor_msgs::Image>("/camera/rgb/image_raw",  1000);
    pubimg_upsample = n.advertise<sensor_msgs::Image>("/camera/xyz/image_upsampling",  1000);
    //  v_ekf = n.advertise<geometry_msgs::PointStamped>("/vekf",100);
    image_save = true;
    std::thread prepro_pc = std::thread(&Preprocess);
    ros::spin();
    //prepro_pc.join();
    return 0;
}