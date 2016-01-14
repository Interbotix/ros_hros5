#include <hros5_camera/hros5_camera.h>
#include <string.h>

int main(int argc, char** argv)
{
    ROS_INFO("Starting hros5_camera_node...");
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    int rate;
    std::string topic;
    nh.param("rate", rate, int(20));
    nh.param("topic", topic,  std::string("/hros5/image"));

    ros::Rate loop_rate(rate);

    hros5::RobotisOPCameraNode camera_node(nh);
    image_transport::ImageTransport it(nh);
    image_transport::CameraPublisher pub = it.advertiseCamera(topic.c_str(), 1);

    dynamic_reconfigure::Server<hros5_camera::hros5_cameraConfig> srv;
    dynamic_reconfigure::Server<hros5_camera::hros5_cameraConfig>::CallbackType cb;
    cb = boost::bind(&hros5::RobotisOPCameraNode::dynamicReconfigureCb,&camera_node, _1, _2);
    srv.setCallback(cb);
    ROS_INFO("hros5_camera_node successfully started");

    while (nh.ok()) {
        camera_node.publishImage(&pub);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

