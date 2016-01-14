

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

//#include <opencv/cv.h>
#include <opencv2/opencv.hpp>

#include <boost/program_options.hpp>
#include <iostream>
using namespace std;
using namespace boost::program_options;

variables_map variablesMap;
int abWidth,abHeight;

cv::Mat resize;
image_transport::Publisher *depthPublisher;

void callback(const sensor_msgs::ImageConstPtr& p_depthFrame)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(p_depthFrame, sensor_msgs::image_encodings::TYPE_16UC1);

    //resize
    //Segfault 14.04 Indigo
    //cv::resize(cv_ptr->image, resize, cv::Size(abHeight,abWidth), 0, 0, CV_INTER_LINEAR);
    resize = cv::Mat::zeros(abHeight, abWidth, CV_16UC1);
    for (int i = 0; i < abHeight; i++)
    {
        for (int j = 0; j < abWidth; j++)
        {
            resize.at<uint16_t>(i, j) = cv_ptr->image.at<uint16_t>((int)(i*p_depthFrame->height/abHeight), (int)(j*p_depthFrame->width/abWidth));
        }
    }

    // convert to ROS image
    cv_bridge::CvImage resizeRos;
    resizeRos.encoding = "16UC1";
    resizeRos.image = resize;
    sensor_msgs::ImagePtr c_newDepthFrame=resizeRos.toImageMsg();
    c_newDepthFrame->width=abWidth;
    c_newDepthFrame->height=abHeight;
    c_newDepthFrame->header.seq = p_depthFrame->header.seq;
    c_newDepthFrame->header.stamp = p_depthFrame->header.stamp;
    c_newDepthFrame->header.frame_id = p_depthFrame->header.frame_id;
    c_newDepthFrame->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    c_newDepthFrame->is_bigendian = 0;

    //republish
    depthPublisher->publish(c_newDepthFrame);
  
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
} 

int main(int argc,char **argv)
{
  string topic_in,topic_out;
  
  // handling arguments
  options_description optionsDescription("Reads image from input topic, resize it and publishes it to output topic.\n"
                                         "To resize set either the absolute or scales width or height for the output");
  optionsDescription.add_options()
    ("help,h","show help message")
    ("topic_in,i", value<string>(&topic_in)->required(),"name of input image topic")
    ("topic_out,o", value<string>(&topic_out)->required(),"name of output image topic")
    ("aw", value<int>(&abWidth),"the absolute width of the output")
    ("ah", value<int>(&abHeight),"the absolute height of the output");

  variables_map variablesMap;
  try
  {
    store(parse_command_line(argc, argv, optionsDescription),variablesMap);
    if (variablesMap.count("help")) {cout<<optionsDescription<<endl; return 0;}
    notify(variablesMap);
  }
  catch (const std::exception& e)
  {
    std::cout << "--------------------" << std::endl;
    std::cerr << "- " << e.what() << std::endl;
    std::cout << "--------------------" << std::endl;
    std::cout << optionsDescription << std::endl;
    return 1;
  }
  
  ros::init(argc, argv, "resize_image");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise(topic_out, 1);
  depthPublisher=&pub;
  image_transport::Subscriber sub = it.subscribe(topic_in, 1, &callback);
  ros::spin();
 
  return 0;
}
