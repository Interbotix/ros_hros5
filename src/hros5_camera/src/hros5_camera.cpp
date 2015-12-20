#include <hros5_camera/hros5_camera.h>
namespace hros5
{
using namespace Robot;

RobotisOPCameraNode::RobotisOPCameraNode(ros::NodeHandle &nh):
    camera_manager_(nh)
{
    // initilize camera
    LinuxCamera::GetInstance()->Initialize(0);
}

RobotisOPCameraNode::~RobotisOPCameraNode(void)
{
}

void RobotisOPCameraNode::publishImage(image_transport::CameraPublisher *pub)
{
    LinuxCamera::GetInstance()->CaptureFrame();

    int pixel_size = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_PixelSize;
    int number_of_pixels = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_NumberOfPixels;

    // write image data to message
    sensor_msgs::Image image_msg;
    image_msg.header.stamp = ros::Time::now();
    image_msg.header.frame_id = "MP_HEAD";
    image_msg.height = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_Height;
    image_msg.width = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_Width;
    image_msg.step = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_WidthStep;
    image_msg.data = std::vector<unsigned char>(LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData,LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData+number_of_pixels*pixel_size);
    image_msg.encoding="rgb8";


    sensor_msgs::CameraInfo image_info = camera_manager_.getCameraInfo();
    image_info.header =  image_msg.header;

    // publish the image
    pub->publish(image_msg,image_info);
}


void RobotisOPCameraNode::dynamicReconfigureCb(hros5_camera::hros5_cameraConfig &config, uint32_t level)
{
    CameraSettings settings;
    settings.brightness = config.brightness;
    settings.exposure = config.exposure;
    settings.gain = config.gain;
    settings.contrast = config.contrast;
    settings.saturation = config.saturation;
    LinuxCamera::GetInstance()->SetCameraSettings(settings);
}

}

