#ifndef SYNC_PROC_SYNC_HPP_
#define SYNC_PROC_SYNC_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/opencv.hpp>
#include <memory>

namespace sync_proc
{

/**
 * @brief RGB-D + IMU 数据同步与预处理节点
 * 
 * 功能:
 * 1. 使用 message_filters 对 RGB、深度、IMU 数据进行时序对齐
 * 2. 应用相机去畸变（基于标定结果）
 * 3. IMU 坐标系变换（从传感器坐标系到机器人坐标系）
 * 4. 发布同步后的数据供 Isaac ROS Visual SLAM 使用
 */
class SyncNode : public rclcpp::Node
{
public:
    explicit SyncNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~SyncNode() = default;

private:
    // 同步策略类型定义
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,     // RGB 图像
        sensor_msgs::msg::Image,     // 深度图像  
        sensor_msgs::msg::Imu        // IMU 数据
    >;

    // 消息订阅器
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    
    // 时间同步器
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    // 发布器
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr synced_rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr synced_depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr synced_imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    
    // TF相关
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 标定参数
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat rectify_map_x_, rectify_map_y_;
    
    // IMU外参变换矩阵 (from calibration)
    geometry_msgs::msg::TransformStamped imu_to_camera_transform_;
    
    // 节点参数
    std::string camera_frame_;
    std::string imu_frame_;
    std::string base_frame_;
    bool enable_rectification_;
    bool enable_imu_transform_;
    double sync_time_tolerance_;
    
    /**
     * @brief 同步回调函数
     * @param rgb_msg RGB图像消息
     * @param depth_msg 深度图像消息  
     * @param imu_msg IMU数据消息
     */
    void syncCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
        const sensor_msgs::msg::Imu::ConstSharedPtr & imu_msg
    );
    
    /**
     * @brief 图像去畸变处理
     * @param input_image 输入图像
     * @return 去畸变后的图像
     */
    cv::Mat rectifyImage(const cv::Mat & input_image);
    
    /**
     * @brief IMU数据坐标变换
     * @param imu_msg 原始IMU数据
     * @return 变换后的IMU数据
     */
    sensor_msgs::msg::Imu transformImuData(const sensor_msgs::msg::Imu & imu_msg);
    
    /**
     * @brief 加载相机标定参数
     */
    void loadCameraCalibration();
    
    /**
     * @brief 加载IMU外参
     */
    void loadImuExtrinsics();
    
    /**
     * @brief 发布相机信息
     */
    void publishCameraInfo(const std_msgs::msg::Header & header);
    
    /**
     * @brief 参数声明和初始化
     */
    void declareParameters();
};

} // namespace sync_proc

#endif // SYNC_PROC_SYNC_HPP_ 