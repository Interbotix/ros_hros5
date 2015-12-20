

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
double scWidth,scHeight;

cv::Mat presize;
cv::Mat resize;
image_transport::Publisher *depthPublisher;
image_transport::Publisher *colorPublisher;
ros::Publisher *cameraInfoPublisher;

void callbacktest(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info, const sensor_msgs::ImageConstPtr& msg)
//void callback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

int source_type = cv_bridge::getCvType(msg->encoding);
int byte_depth = sensor_msgs::image_encodings::bitDepth(msg->encoding) / 8;
int num_channels = sensor_msgs::image_encodings::numChannels(msg->encoding);

presize = cv::Mat(msg->height, msg->width, CV_MAKETYPE(CV_8U, byte_depth*num_channels), const_cast<uchar*>(&msg->data[0]), msg->step);


//cv::Mat resize(480, 640, CV_16UC1);
      //resize = cv_bridge::matFromImage( msg );

// compute new size
abWidth = 640;
abHeight = 480;
//printf( "width %d, height %d\r\n", abWidth, abHeight );

resize = cv::Mat::zeros(abHeight, abWidth, CV_16UC1);
//printf( "zeros ok\r\n" );

//    cv::resize(presize, resize, cv::Size(abHeight,abWidth), 0, 0, CV_INTER_LINEAR);
//printf( "resize ok\r\n" );

// create a gradient, just to see something meaningfull
for (int i = 0; i < 480; i++)
{
    for (int j = 0; j < 640; j++)
    {
        /*int gradientInY = (i / 480.f) * 65535;
        image.at<uint16_t>(i, j) = gradientInY;*/

        //int gradientInX = (j / 640.f) * 65535;
        //resize.at<uint16_t>(i, j) = gradientInX;
        resize.at<uint16_t>(i, j) = presize.at<uint16_t>(int(i*0.75), int(j*0.75));
    }
}







/*
sensor_msgs::CameraInfo info_left;
info_left.header.stamp = ros::Time::now();
*/
cameraInfoPublisher->publish(cam_info);






colorPublisher->publish(image);








    // convert to Ros image
    cv_bridge::CvImage resizeRos;
    resizeRos.encoding = "16UC1";
    resizeRos.image = resize;
    // republish image
    sensor_msgs::ImagePtr imagePtr=resizeRos.toImageMsg();
    imagePtr->width=abWidth;
    imagePtr->height=abHeight;
    depthPublisher->publish(imagePtr);



    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }





  /*
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    // convert to OpenCV Mat
    //cv_ptr=cv_bridge::toCvShare(msg,"bgr8");

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

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
    depthPublisher->publish(imagePtr);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  */
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
  





  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/color/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, "/camera/color/camera_info", 1);
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image> sync(image_sub, info_sub, depth_sub, 10);
  sync.registerCallback(boost::bind(&callbacktest, _1, _2, _3));

  image_transport::ImageTransport it(nh);
  image_transport::Publisher      pub = it.advertise("/test/depth/image_resized" /*topic_out*/, 1);
  depthPublisher = &pub;

  image_transport::Publisher      pub2 = it.advertise("/test/color/image_raw", 1);
  colorPublisher = &pub2;

  ros::Publisher pub3 = nh.advertise<sensor_msgs::CameraInfo>("/test/color/camera_info", 1);
  cameraInfoPublisher = &pub3;


  /*
  image_transport::ImageTransport it(nh);
  image_transport::Publisher      pub = it.advertise(topic_out, 1);
  depthPublisher = &pub;

  image_transport::Subscriber sub = it.subscribe(topic_in, 1, &callback);
  */


  ros::spin();

  return 0;
}
