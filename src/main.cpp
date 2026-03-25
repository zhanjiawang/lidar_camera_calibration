#include <fcntl.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <iostream>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "CustomMsg.h"
#include "ros/ros.h"

std::atomic<bool> is_get_image_(false);
std::atomic<bool> is_get_lidar_(false);
cv::Mat get_image_;
int get_lidar_cloud_count_ = 0;
pcl::PointCloud<pcl::PointXYZI>::Ptr get_lidar_cloud_(
    new pcl::PointCloud<pcl::PointXYZI>);

std::string lidar_msg_type_ = "";
std::string lidar_msg_name_ = "/livox/lidar";
std::string image_msg_name_ = "/camera/color/image_raw";
float point_cloud_max_distance_ = 3.0;
float point_cloud_min_distance_ = 0.3;
float point_cloud_horizontal_min_fov_ = -45.0;
float point_cloud_horizontal_max_fov_ = 45.0;
float point_cloud_vertical_min_fov_ = -60.0;
float point_cloud_vertical_max_fov_ = 30.0;
int point_cloud_cumulative_number_ = 20;
Eigen::Matrix3f camera_intrinsic_parameters_matrix_ =
    Eigen::Matrix3f::Identity();
Eigen::Matrix4f image_coordinate_system_matrix_ = Eigen::Matrix4f::Identity();
std::vector<double> initial_extrinsic_parameters_;
std::vector<double> increment_extrinsic_parameters_;

int color_num_haploid_ = 256;
int color_num_double_ = 512;
int color_num_triple_ = 768;
int color_num_quadruple_ = 1024;
int color_num_penta_ = 1280;
std::vector<int> color_red_palette_;
std::vector<int> color_green_palette_;
std::vector<int> color_blue_palette_;

ros::Subscriber sub_image_;
ros::Subscriber sub_lidar_;
ros::Subscriber sub_extrinsic_parameters_;

ros::Publisher pub_image_;
ros::Publisher pub_lidar_;
ros::Publisher pub_color_lidar_;

// ros图像转opencv图像
void RosImageToOpenCVImage(sensor_msgs::Image &image_source,
                           cv::Mat &image_target) {
  image_target = cv::Mat(image_source.height, image_source.width, CV_8UC3);
  memcpy(image_target.data, image_source.data.data(), image_source.data.size());
}

// opencv图像转ros图像
void OpenCVImageToRosImage(cv::Mat &image_source,
                           sensor_msgs::Image &image_target,
                           std::string encoding) {
  image_target.height = image_source.rows;
  image_target.width = image_source.cols;
  image_target.encoding = encoding;
  image_target.is_bigendian = 0;
  image_target.step = image_source.cols * image_source.elemSize();
  size_t size = image_target.step * image_source.rows;
  image_target.data.resize(size);
  if (image_source.isContinuous()) {
    memcpy((char *)(&image_target.data[0]), image_source.data, size);
  } else {
    uchar *ros_data_ptr = (uchar *)(&image_target.data[0]);
    uchar *cv_data_ptr = image_source.data;
    for (int i = 0; i < image_source.rows; ++i) {
      memcpy(ros_data_ptr, cv_data_ptr, image_target.step);
      ros_data_ptr += image_target.step;
      cv_data_ptr += image_source.step;
    }
  }
}

//准备调色盘
void PrepareColorPalette() {
  int red, green, blue;
  for (int i = 0; i <= color_num_penta_; i++) {
    int intensity = color_num_penta_ - i;
    if (intensity < color_num_haploid_) {
      red = 255;
      green = 255 * intensity / color_num_haploid_;
      blue = 0;
    } else if (intensity < color_num_double_) {
      red = 255 * (color_num_double_ - intensity) / color_num_haploid_;
      green = 255;
      blue = 0;
    } else if (intensity < color_num_triple_) {
      red = 0;
      green = 255;
      blue = 255 * (intensity - color_num_double_) / color_num_haploid_;
    } else if (intensity < color_num_quadruple_) {
      red = 0;
      green = 255 * (color_num_quadruple_ - intensity) / color_num_haploid_;
      blue = 255;
    } else {
      continue;
    }
    color_red_palette_.push_back(red);
    color_green_palette_.push_back(green);
    color_blue_palette_.push_back(blue);
  }
}

//图像回调函数
void ImageCallback(const sensor_msgs::Image::ConstPtr &msgIn) {
  if (is_get_image_ == false) {
    sensor_msgs::Image image_msg = *msgIn;
    RosImageToOpenCVImage(image_msg, get_image_);
    cv::imwrite(
        "/home/zhanjiawang/CodeFiles/lidar_camera_calibration/output.png",
        get_image_);
    PrepareColorPalette();
    sensor_msgs::Image ros_image;
    OpenCVImageToRosImage(get_image_, ros_image, "bgr8");
    ros_image.header.stamp = ros::Time::now();
    pub_image_.publish(ros_image);
    is_get_image_ = true;
  }
}

void ProcessLidar(pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud) {
  if (is_get_lidar_ == false) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr part_lidar_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);

    for (auto &point : *lidar_cloud) {
      float distance =
          sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
      float horizontal_angel = atan2(point.y, point.x);
      float vertical_angel =
          atan2(point.z, sqrt(pow(point.x, 2) + pow(point.y, 2)));
      horizontal_angel = 180.0 * horizontal_angel / M_PI;
      vertical_angel = 180.0 * vertical_angel / M_PI;
      if (distance > point_cloud_min_distance_ &&
          distance < point_cloud_max_distance_ &&
          horizontal_angel > point_cloud_horizontal_min_fov_ &&
          horizontal_angel < point_cloud_horizontal_max_fov_ &&
          vertical_angel > point_cloud_vertical_min_fov_ &&
          vertical_angel < point_cloud_vertical_max_fov_) {
        (*part_lidar_cloud).push_back(point);
      }
    }

    (*get_lidar_cloud_) += (*part_lidar_cloud);

    get_lidar_cloud_count_++;
    if (get_lidar_cloud_count_ > point_cloud_cumulative_number_) {
      sensor_msgs::PointCloud2 get_lidar_cloud_msg;
      get_lidar_cloud_msg.header.stamp = ros::Time::now();
      pcl::toROSMsg(*get_lidar_cloud_, get_lidar_cloud_msg);
      get_lidar_cloud_msg.header.frame_id = "map";
      pub_lidar_.publish(get_lidar_cloud_msg);
      is_get_lidar_ = true;
      std::cout << "数据准备完成，可以调整标定参数!" << std::endl;
    }
  }
}

//点云回调函数
void PointCloud2LidarCallback(const sensor_msgs::PointCloud2::ConstPtr msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *lidar_cloud);
  ProcessLidar(lidar_cloud);
}

//点云回调函数
void CustomMsgLidarCallback(const livox_ros_driver::CustomMsgConstPtr msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  for (unsigned int i = 0; i < msg->point_num; ++i) {
    pcl::PointXYZI pointxyzi;
    pointxyzi.x = msg->points[i].x;
    pointxyzi.y = msg->points[i].y;
    pointxyzi.z = msg->points[i].z;
    pointxyzi.intensity = ((int)msg->points[i].reflectivity) / 255.0;
    (*lidar_cloud).push_back(pointxyzi);
  }
  ProcessLidar(lidar_cloud);
}

// 设置终端为非阻塞模式
void SetNonblockingMode(int enable) {
  static struct termios original_termios;
  static int original_flags;
  static int initialized = 0;

  if (!initialized) {
    tcgetattr(STDIN_FILENO, &original_termios);
    original_flags = fcntl(STDIN_FILENO, F_GETFL);
    initialized = 1;
  }

  if (enable) {
    struct termios new_termios = original_termios;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
    fcntl(STDIN_FILENO, F_SETFL, original_flags | O_NONBLOCK);
  } else {
    tcsetattr(STDIN_FILENO, TCSANOW, &original_termios);
    fcntl(STDIN_FILENO, F_SETFL, original_flags);
  }
}

//使用外参投影点云为图像
void ExtrinsicParametersProjection() {
  if (is_get_lidar_ == true && is_get_image_ == true) {
    std::cout << initial_extrinsic_parameters_[0] << " "
              << initial_extrinsic_parameters_[1] << " "
              << initial_extrinsic_parameters_[2] << " "
              << initial_extrinsic_parameters_[3] << " "
              << initial_extrinsic_parameters_[4] << " "
              << initial_extrinsic_parameters_[5] << std::endl;

    float roll = M_PI * initial_extrinsic_parameters_[0] / 180.0;
    float pitch = M_PI * initial_extrinsic_parameters_[1] / 180.0;
    float yaw = M_PI * initial_extrinsic_parameters_[2] / 180.0;
    float x = initial_extrinsic_parameters_[3];
    float y = initial_extrinsic_parameters_[4];
    float z = initial_extrinsic_parameters_[5];
    Eigen::AngleAxisf roll_angle_axis(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitch_angle_axis(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yaw_angle_axis(yaw, Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f rotation =
        (yaw_angle_axis * pitch_angle_axis * roll_angle_axis)
            .toRotationMatrix();
    Eigen::Vector3f translation = Eigen::Vector3f(x, y, z);
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    transformation.block<3, 3>(0, 0) = rotation;
    transformation.block<3, 1>(0, 3) = translation;

    std::cout << transformation << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr get_lidar_cloud_transformed(
        new pcl::PointCloud<pcl::PointXYZI>);
    transformation = image_coordinate_system_matrix_ * transformation;
    pcl::transformPointCloud(*get_lidar_cloud_, *get_lidar_cloud_transformed,
                             transformation);

    int image_width = get_image_.cols;
    int image_height = get_image_.rows;
    cv::Mat image_lidar =
        cv::Mat(image_height, image_width, CV_8UC3, cv::Scalar(0, 0, 0));

    float min_distance = FLT_MAX;
    float max_distance = -FLT_MAX;
    for (auto &pt : get_lidar_cloud_transformed->points) {
      float distance = sqrt(pow(pt.x, 2) + pow(pt.y, 2) + pow(pt.z, 2));
      if (distance < min_distance) {
        min_distance = distance;
      }
      if (distance > max_distance) {
        max_distance = distance;
      }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_lidar_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    int pt_index = 0;
    for (auto &pt : get_lidar_cloud_transformed->points) {
      float distance = sqrt(pow(pt.x, 2) + pow(pt.y, 2) + pow(pt.z, 2));
      int color_index = color_num_penta_ * (distance - min_distance) /
                        (max_distance - min_distance);
      if (color_index >= 0 && color_index < color_num_penta_) {
        int red = color_red_palette_[color_index];
        int green = color_green_palette_[color_index];
        int blue = color_blue_palette_[color_index];
        Eigen::Vector3f point_2d =
            Eigen::Vector3f(pt.x / pt.z, pt.y / pt.z, 1.0);
        point_2d = camera_intrinsic_parameters_matrix_ * point_2d;
        if (point_2d[0] >= 0 && point_2d[0] < image_width && point_2d[1] >= 0 &&
            point_2d[1] < image_height) {
          pcl::PointXYZRGB point_xyzrgb;
          point_xyzrgb.x = (*get_lidar_cloud_)[pt_index].x;
          point_xyzrgb.y = (*get_lidar_cloud_)[pt_index].y;
          point_xyzrgb.z = (*get_lidar_cloud_)[pt_index].z;
          cv::Vec3b pixel = get_image_.at<cv::Vec3b>(point_2d[1], point_2d[0]);
          point_xyzrgb.r = pixel[0];
          point_xyzrgb.g = pixel[1];
          point_xyzrgb.b = pixel[2];
          (*color_lidar_cloud).push_back(point_xyzrgb);
          for (int m = -2; m <= 2; m++) {
            for (int n = -2; n <= 2; n++) {
              int pixel_x = point_2d[0] + m;
              int pixel_y = point_2d[1] + n;
              if (pixel_x >= 0 && pixel_x < image_width && pixel_y >= 0 &&
                  pixel_y < image_height) {
                image_lidar.at<cv::Vec3b>(pixel_y, pixel_x) =
                    cv::Vec3b(blue, green, red);
              }
            }
          }
        }
      }
      pt_index++;
    }

    sensor_msgs::PointCloud2 color_lidar_cloud_msg;
    color_lidar_cloud_msg.header.stamp = ros::Time::now();
    pcl::toROSMsg(*color_lidar_cloud, color_lidar_cloud_msg);
    color_lidar_cloud_msg.header.frame_id = "map";
    pub_color_lidar_.publish(color_lidar_cloud_msg);

    //渲染结果
    cv::addWeighted(get_image_, 0.7, image_lidar, 0.3, 0.0, image_lidar);

    sensor_msgs::Image ros_image;
    OpenCVImageToRosImage(image_lidar, ros_image, "bgr8");
    ros_image.header.stamp = ros::Time::now();
    pub_image_.publish(ros_image);
  }
}

//调整外参并得到投影结果
void ProjectionExploration() {
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << "按q增加roll  按a减少roll" << std::endl;
  std::cout << "按w增加pitch 按s减少pitch" << std::endl;
  std::cout << "按e增加yaw   按d减少yaw" << std::endl;
  std::cout << "按r增加x     按f减少x" << std::endl;
  std::cout << "按t增加y     按g减少y" << std::endl;
  std::cout << "按y增加z     按h减少z" << std::endl;
  std::cout << "非阻塞键盘读取演示 (按空格退出)" << std::endl;
  SetNonblockingMode(1);

  while (true) {
    int input = getchar();
    if (input != EOF) {
      char char_input = input;
      if (input == ' ') {
        break;
      } else if (input == 'q') {
        std::cout << "增加roll" << std::endl;
        initial_extrinsic_parameters_[0] += increment_extrinsic_parameters_[0];
        ExtrinsicParametersProjection();
      } else if (input == 'a') {
        std::cout << "减少roll" << std::endl;
        initial_extrinsic_parameters_[0] -= increment_extrinsic_parameters_[0];
        ExtrinsicParametersProjection();
      } else if (input == 'w') {
        std::cout << "增加pitch" << std::endl;
        initial_extrinsic_parameters_[1] += increment_extrinsic_parameters_[1];
        ExtrinsicParametersProjection();
      } else if (input == 's') {
        std::cout << "减少pitch" << std::endl;
        initial_extrinsic_parameters_[1] -= increment_extrinsic_parameters_[1];
        ExtrinsicParametersProjection();
      } else if (input == 'e') {
        std::cout << "增加yaw" << std::endl;
        initial_extrinsic_parameters_[2] += increment_extrinsic_parameters_[2];
        ExtrinsicParametersProjection();
      } else if (input == 'd') {
        std::cout << "减少yaw" << std::endl;
        initial_extrinsic_parameters_[2] -= increment_extrinsic_parameters_[2];
        ExtrinsicParametersProjection();
      } else if (input == 'r') {
        std::cout << "增加x" << std::endl;
        initial_extrinsic_parameters_[3] += increment_extrinsic_parameters_[3];
        ExtrinsicParametersProjection();
      } else if (input == 'f') {
        std::cout << "减少x" << std::endl;
        initial_extrinsic_parameters_[3] -= increment_extrinsic_parameters_[3];
        ExtrinsicParametersProjection();
      } else if (input == 't') {
        std::cout << "增加y" << std::endl;
        initial_extrinsic_parameters_[4] += increment_extrinsic_parameters_[4];
        ExtrinsicParametersProjection();
      } else if (input == 'g') {
        std::cout << "减少y" << std::endl;
        initial_extrinsic_parameters_[4] -= increment_extrinsic_parameters_[4];
        ExtrinsicParametersProjection();
      } else if (input == 'y') {
        std::cout << "增加z" << std::endl;
        initial_extrinsic_parameters_[5] += increment_extrinsic_parameters_[5];
        ExtrinsicParametersProjection();
      } else if (input == 'h') {
        std::cout << "减少z" << std::endl;
        initial_extrinsic_parameters_[5] -= increment_extrinsic_parameters_[5];
        ExtrinsicParametersProjection();
      }
    }
    usleep(100000);
  }

  SetNonblockingMode(0);
  std::cout << "程序结束" << std::endl;
}

void ReadParam(ros::NodeHandle &nh) {
  nh.param<std::string>("lidar_camera_calibration/lidar_msg_type",
                        lidar_msg_type_, "");
  nh.param<std::string>("lidar_camera_calibration/lidar_msg_name",
                        lidar_msg_name_, "/livox/lidar");
  nh.param<std::string>("lidar_camera_calibration/image_msg_name",
                        image_msg_name_, "/camera/color/image_raw");
  nh.param<float>("lidar_camera_calibration/point_cloud_max_distance",
                  point_cloud_max_distance_, 3.0);
  nh.param<float>("lidar_camera_calibration/point_cloud_min_distance",
                  point_cloud_min_distance_, 0.3);
  nh.param<float>("lidar_camera_calibration/point_cloud_horizontal_min_fov",
                  point_cloud_horizontal_min_fov_, -45.0);
  nh.param<float>("lidar_camera_calibration/point_cloud_horizontal_max_fov",
                  point_cloud_horizontal_max_fov_, 45.0);
  nh.param<float>("lidar_camera_calibration/point_cloud_vertical_min_fov",
                  point_cloud_vertical_min_fov_, -60.0);
  nh.param<float>("lidar_camera_calibration/point_cloud_vertical_max_fov",
                  point_cloud_vertical_max_fov_, 30.0);
  nh.param<int>("lidar_camera_calibration/point_cloud_cumulative_number",
                point_cloud_cumulative_number_, 20);
  std::vector<float> camera_intrinsic_parameters;
  nh.param<std::vector<float>>(
      "lidar_camera_calibration/camera_intrinsic_parameters",
      camera_intrinsic_parameters, std::vector<float>());
  nh.param<std::vector<double>>(
      "lidar_camera_calibration/initial_extrinsic_parameters",
      initial_extrinsic_parameters_, std::vector<double>());
  nh.param<std::vector<double>>(
      "lidar_camera_calibration/increment_extrinsic_parameters",
      increment_extrinsic_parameters_, std::vector<double>());

  std::cout << "lidar_msg_type: " << lidar_msg_type_ << std::endl;
  std::cout << "lidar_msg_name: " << lidar_msg_name_ << std::endl;
  std::cout << "image_msg_name: " << image_msg_name_ << std::endl;
  std::cout << "point_cloud_max_distance: " << point_cloud_max_distance_
            << std::endl;
  std::cout << "point_cloud_min_distance: " << point_cloud_min_distance_
            << std::endl;
  std::cout << "point_cloud_horizontal_min_fov: "
            << point_cloud_horizontal_min_fov_ << std::endl;
  std::cout << "point_cloud_horizontal_max_fov: "
            << point_cloud_horizontal_max_fov_ << std::endl;
  std::cout << "point_cloud_vertical_min_fov: " << point_cloud_vertical_min_fov_
            << std::endl;
  std::cout << "point_cloud_vertical_max_fov: " << point_cloud_vertical_max_fov_
            << std::endl;
  std::cout << "point_cloud_cumulative_number: "
            << point_cloud_cumulative_number_ << std::endl;
  camera_intrinsic_parameters_matrix_ << camera_intrinsic_parameters[0],
      camera_intrinsic_parameters[1], camera_intrinsic_parameters[2],
      camera_intrinsic_parameters[3], camera_intrinsic_parameters[4],
      camera_intrinsic_parameters[5], camera_intrinsic_parameters[6],
      camera_intrinsic_parameters[7], camera_intrinsic_parameters[8];
  std::cout << "camera_intrinsic_parameters_matrix: " << std::endl;
  std::cout << camera_intrinsic_parameters_matrix_ << std::endl;
  std::cout << "initial_extrinsic_parameters: " << std::endl;
  for (int i = 0; i < initial_extrinsic_parameters_.size(); i++) {
    std::cout << initial_extrinsic_parameters_[i] << " ";
  }
  std::cout << std::endl;
  std::cout << "increment_extrinsic_parameters: " << std::endl;
  for (int i = 0; i < increment_extrinsic_parameters_.size(); i++) {
    std::cout << increment_extrinsic_parameters_[i] << " ";
  }
  std::cout << std::endl;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "lidar_camera_calibration");
  ros::NodeHandle nh;

  ReadParam(nh);

  image_coordinate_system_matrix_ << 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  sub_image_ =
      nh.subscribe<sensor_msgs::Image>(image_msg_name_, 10, ImageCallback);
  if (lidar_msg_type_ == "PointCloud2") {
    sub_lidar_ = nh.subscribe<sensor_msgs::PointCloud2>(
        lidar_msg_name_, 10, PointCloud2LidarCallback);
  } else if (lidar_msg_type_ == "CustomMsg") {
    sub_lidar_ = nh.subscribe<livox_ros_driver::CustomMsg>(
        lidar_msg_name_, 10, CustomMsgLidarCallback);
  } else {
    std::cout << "not support lidar msg type" << std::endl;
  }

  pub_image_ = nh.advertise<sensor_msgs::Image>(
      "/lidar_camera_calibration/image", 1, true);
  pub_lidar_ = nh.advertise<sensor_msgs::PointCloud2>(
      "/lidar_camera_calibration/lidar", 1, true);
  pub_color_lidar_ = nh.advertise<sensor_msgs::PointCloud2>(
      "/lidar_camera_calibration/color_lidar", 1, true);

  std::thread projection_exploration_thread =
      std::thread(ProjectionExploration);

  ros::spin();

  projection_exploration_thread.join();

  return 0;
}