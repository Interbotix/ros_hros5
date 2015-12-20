

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>

#include <boost/program_options.hpp>
#include <iostream>
using namespace std;
using namespace boost::program_options;

variables_map variablesMap;
int abWidth,abHeight;
double scWidth,scHeight;

cv::Mat resize;
image_transport::Publisher *imagePub;

void callback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    // convert to OpenCV Mat
    cv_ptr=cv_bridge::toCvShare(msg,"bgr8");
    // compute new size
    if (!variablesMap.count("aw"))
      abWidth=cv_ptr->image.size().width*scWidth+0.5;
    if (!variablesMap.count("ah"))
      abHeight=cv_ptr->image.size().height*scHeight+0.5;
    // resize
    cv::resize(cv_ptr->image,resize, cv::Size(abWidth,abHeight));
    // convert to Ros image
    cv_bridge::CvImage resizeRos;
    resizeRos.encoding = "bgr8";
    resizeRos.image = resize;
    // republish image
    sensor_msgs::ImagePtr imagePtr=resizeRos.toImageMsg();
    imagePtr->width=abWidth;
    imagePtr->height=abHeight;
    imagePub->publish(imagePtr);
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
    ("ah", value<int>(&abHeight),"the absolute height of the output")
    ("sw", value<double>(&scWidth)->default_value(1.0),"the width scaling factor")
    ("sh", value<double>(&scHeight)->default_value(1.0),"the height scaling factor");

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
  imagePub=&pub;
  image_transport::Subscriber sub = it.subscribe(topic_in, 1, &callback);
  ros::spin();

  return 0;
}
