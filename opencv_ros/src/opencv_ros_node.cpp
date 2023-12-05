#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>


static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv::Ptr<cv::SimpleBlobDetector> detector;


public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/iiwa/camera1/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);

  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Converti l'immagine in scala di grigi
    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

    // Esegui la rilevazione dei cerchi utilizzando la trasformata di Hough
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, gray.rows / 8, 100, 30, 0, 0);

    // Disegna i cerchi rilevati sull'immagine
    for (size_t i = 0; i < circles.size(); i++)
    {
      cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      // Disegna il cerchio sul frame originale
      cv::circle(cv_ptr->image, center, radius, cv::Scalar(0, 255, 0), 3);
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}