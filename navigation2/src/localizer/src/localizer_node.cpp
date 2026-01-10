#include <queue>
#include <mutex>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "localizers/commons.h"
#include "localizers/icp_localizer.h"
#include "interface/srv/relocalize.hpp"
#include "interface/srv/is_valid.hpp"
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

struct NodeConfig
{
    std::string cloud_topic = "/fastlio2/body_cloud";
    std::string odom_topic = "/fastlio2/lio_odom";
    std::string map_frame = "map";
    std::string local_frame = "lidar";
    double update_hz = 1.0;
    
    // 滤波器配置参数
    double filter_alpha = 0.3;  // EWMA滤波系数，范围[0,1]，越小越平滑
    bool enable_odom_filter = true;  // 是否启用里程计滤波
};

struct NodeState
{
    std::mutex message_mutex;
    std::mutex service_mutex;

    bool message_received = false;
    bool service_received = false;
    bool localize_success = false;
    rclcpp::Time last_send_tf_time = rclcpp::Clock().now();
    builtin_interfaces::msg::Time last_message_time;
    CloudType::Ptr last_cloud = std::make_shared<CloudType>();
    M3D last_r;                          // localmap_body_r
    V3D last_t;                          // localmap_body_t
    M3D last_offset_r = M3D::Identity(); // map_localmap_r
    V3D last_offset_t = V3D::Zero();     // map_localmap_t
    M4F initial_guess = M4F::Identity();
    
    // 滤波器状态变量
    bool filter_initialized = false;
    V3D filtered_position = V3D::Zero();      // 滤波后的位置
    V3D filtered_linear_vel = V3D::Zero();    // 滤波后的线速度
    V3D filtered_angular_vel = V3D::Zero();   // 滤波后的角速度
    Eigen::Quaterniond filtered_orientation = Eigen::Quaterniond::Identity();  // 滤波后的姿态
};

class LocalizerNode : public rclcpp::Node
{
public:
    LocalizerNode() : Node("localizer_node")
    {
        RCLCPP_INFO(this->get_logger(), "Localizer Node Started");
        
        // 创建回调组
        m_timer_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        m_subscriber_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        m_service_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // 创建执行器选项
        rclcpp::SubscriptionOptions subscription_options;
        subscription_options.callback_group = m_subscriber_callback_group;

        rclcpp::PublisherOptions publisher_options;
        publisher_options.callback_group = m_subscriber_callback_group;

        loadParameters();
        rclcpp::QoS qos = rclcpp::QoS(10);
        
        // 使用选项创建订阅者
        m_cloud_sub.subscribe(this, m_config.cloud_topic, qos.get_rmw_qos_profile());
        m_odom_sub.subscribe(this, m_config.odom_topic, qos.get_rmw_qos_profile());

        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10), m_cloud_sub, m_odom_sub);
        m_sync->setAgePenalty(0.1);
        m_sync->registerCallback(std::bind(&LocalizerNode::syncCB, this, std::placeholders::_1, std::placeholders::_2));
        m_localizer = std::make_shared<ICPLocalizer>(m_localizer_config);

        // 使用回调组创建服务
        m_reloc_srv = this->create_service<interface::srv::Relocalize>(
            "relocalize", 
            std::bind(&LocalizerNode::relocCB, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            m_service_callback_group);

        m_reloc_check_srv = this->create_service<interface::srv::IsValid>(
            "relocalize_check", 
            std::bind(&LocalizerNode::relocCheckCB, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            m_service_callback_group);

        m_map_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", 10);

        // 创建滤波后的里程计发布器，供Navigation2使用
        m_filtered_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // 使用回调组创建定时器
        m_timer = this->create_wall_timer(
            10ms, 
            std::bind(&LocalizerNode::timerCB, this),
            m_timer_callback_group);
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);
        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        m_config.cloud_topic = config["cloud_topic"].as<std::string>();
        m_config.odom_topic = config["odom_topic"].as<std::string>();
        m_config.map_frame = config["map_frame"].as<std::string>();
        m_config.local_frame = config["local_frame"].as<std::string>();
        m_config.update_hz = config["update_hz"].as<double>();

        // 加载滤波器配置参数
        if (config["filter_alpha"]) {
            m_config.filter_alpha = config["filter_alpha"].as<double>();
        }
        if (config["enable_odom_filter"]) {
            m_config.enable_odom_filter = config["enable_odom_filter"].as<bool>();
        }

        m_localizer_config.rough_scan_resolution = config["rough_scan_resolution"].as<double>();
        m_localizer_config.rough_map_resolution = config["rough_map_resolution"].as<double>();
        m_localizer_config.rough_max_iteration = config["rough_max_iteration"].as<int>();
        m_localizer_config.rough_score_thresh = config["rough_score_thresh"].as<double>();

        m_localizer_config.refine_scan_resolution = config["refine_scan_resolution"].as<double>();
        m_localizer_config.refine_map_resolution = config["refine_map_resolution"].as<double>();
        m_localizer_config.refine_max_iteration = config["refine_max_iteration"].as<int>();
        m_localizer_config.refine_score_thresh = config["refine_score_thresh"].as<double>();
    }

    void timerCB()
    {
        if (!m_state.message_received)
            return;

        rclcpp::Duration diff = rclcpp::Clock().now() - m_state.last_send_tf_time;

        bool update_tf = diff.seconds() > (1.0 / m_config.update_hz) && m_state.message_received;

        if (!update_tf)
        {
            sendBroadCastTF(m_state.last_message_time);
            return;
        }

        m_state.last_send_tf_time = rclcpp::Clock().now();

        M4F initial_guess = M4F::Identity();
        if (m_state.service_received)
        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            initial_guess = m_state.initial_guess;
            // m_state.service_received = false;
        }
        else
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            initial_guess.block<3, 3>(0, 0) = (m_state.last_offset_r * m_state.last_r).cast<float>();
            initial_guess.block<3, 1>(0, 3) = (m_state.last_offset_r * m_state.last_t + m_state.last_offset_t).cast<float>();
        }

        M3D current_local_r;
        V3D current_local_t;
        builtin_interfaces::msg::Time current_time;
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            current_local_r = m_state.last_r;
            current_local_t = m_state.last_t;
            current_time = m_state.last_message_time;
            m_localizer->setInput(m_state.last_cloud);
        }

        bool result = m_localizer->align(initial_guess);
        if (result)
        {
            M3D map_body_r = initial_guess.block<3, 3>(0, 0).cast<double>();
            V3D map_body_t = initial_guess.block<3, 1>(0, 3).cast<double>();
            m_state.last_offset_r = map_body_r * current_local_r.transpose();
            m_state.last_offset_t = -map_body_r * current_local_r.transpose() * current_local_t + map_body_t;
            if (!m_state.localize_success && m_state.service_received)
            {
                std::lock_guard<std::mutex> lock(m_state.service_mutex);
                m_state.localize_success = true;
                m_state.service_received = false;
            }
        }
        sendBroadCastTF(current_time);
        publishMapCloud(current_time);
    }

    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {
        std::lock_guard<std::mutex> lock(m_state.message_mutex);

        pcl::fromROSMsg(*cloud_msg, *m_state.last_cloud);

        m_state.last_r = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                            odom_msg->pose.pose.orientation.x,
                                            odom_msg->pose.pose.orientation.y,
                                            odom_msg->pose.pose.orientation.z)
                             .toRotationMatrix();
        m_state.last_t = V3D(odom_msg->pose.pose.position.x,
                             odom_msg->pose.pose.position.y,
                             odom_msg->pose.pose.position.z);
        m_state.last_message_time = cloud_msg->header.stamp;
        if (!m_state.message_received)
        {
            m_state.message_received = true;
            m_config.local_frame = odom_msg->header.frame_id;
        }
        
        // 对里程计数据进行滤波并发布
        if (m_config.enable_odom_filter)
        {
            applyOdomFilter(odom_msg);
        }
    }
    
    // 使用指数加权移动平均（EWMA）滤波器平滑里程计数据
    void applyOdomFilter(const nav_msgs::msg::Odometry::ConstSharedPtr &raw_odom)
    {
        const double alpha = m_config.filter_alpha;  // 滤波系数
        
        // 提取原始数据
        V3D raw_position(raw_odom->pose.pose.position.x,
                         raw_odom->pose.pose.position.y,
                         raw_odom->pose.pose.position.z);
        
        Eigen::Quaterniond raw_orientation(raw_odom->pose.pose.orientation.w,
                                          raw_odom->pose.pose.orientation.x,
                                          raw_odom->pose.pose.orientation.y,
                                          raw_odom->pose.pose.orientation.z);
        
        V3D raw_linear_vel(raw_odom->twist.twist.linear.x,
                          raw_odom->twist.twist.linear.y,
                          raw_odom->twist.twist.linear.z);
        
        V3D raw_angular_vel(raw_odom->twist.twist.angular.x,
                           raw_odom->twist.twist.angular.y,
                           raw_odom->twist.twist.angular.z);
        
        // 初始化滤波器
        if (!m_state.filter_initialized)
        {
            m_state.filtered_position = raw_position;
            m_state.filtered_orientation = raw_orientation;
            m_state.filtered_linear_vel = raw_linear_vel;
            m_state.filtered_angular_vel = raw_angular_vel;
            m_state.filter_initialized = true;
        }
        else
        {
            // EWMA滤波：filtered = alpha * raw + (1 - alpha) * filtered_prev
            m_state.filtered_position = alpha * raw_position + (1.0 - alpha) * m_state.filtered_position;
            
            // 四元数球面线性插值（SLERP）
            m_state.filtered_orientation = m_state.filtered_orientation.slerp(alpha, raw_orientation);
            
            // 速度滤波
            m_state.filtered_linear_vel = alpha * raw_linear_vel + (1.0 - alpha) * m_state.filtered_linear_vel;
            m_state.filtered_angular_vel = alpha * raw_angular_vel + (1.0 - alpha) * m_state.filtered_angular_vel;
        }
        
        // 发布滤波后的里程计数据
        publishFilteredOdom(raw_odom->header);
    }
    
    // 发布滤波后的里程计数据供Navigation2使用
    void publishFilteredOdom(const std_msgs::msg::Header &header)
    {
        if (m_filtered_odom_pub->get_subscription_count() < 1)
            return;
        
        nav_msgs::msg::Odometry filtered_odom;
        filtered_odom.header = header;
        filtered_odom.header.frame_id = m_config.local_frame;  // 使用lidar坐标系
        filtered_odom.child_frame_id = "base_link";
        
        // 设置滤波后的位置
        filtered_odom.pose.pose.position.x = m_state.filtered_position.x();
        filtered_odom.pose.pose.position.y = m_state.filtered_position.y();
        filtered_odom.pose.pose.position.z = m_state.filtered_position.z();
        
        // 设置滤波后的姿态
        filtered_odom.pose.pose.orientation.w = m_state.filtered_orientation.w();
        filtered_odom.pose.pose.orientation.x = m_state.filtered_orientation.x();
        filtered_odom.pose.pose.orientation.y = m_state.filtered_orientation.y();
        filtered_odom.pose.pose.orientation.z = m_state.filtered_orientation.z();
        
        // 设置滤波后的速度
        filtered_odom.twist.twist.linear.x = m_state.filtered_linear_vel.x();
        filtered_odom.twist.twist.linear.y = m_state.filtered_linear_vel.y();
        filtered_odom.twist.twist.linear.z = m_state.filtered_linear_vel.z();
        
        filtered_odom.twist.twist.angular.x = m_state.filtered_angular_vel.x();
        filtered_odom.twist.twist.angular.y = m_state.filtered_angular_vel.y();
        filtered_odom.twist.twist.angular.z = m_state.filtered_angular_vel.z();
        
        // 设置协方差（比原始数据更小，表示更稳定）
        for (int i = 0; i < 36; i++)
        {
            filtered_odom.pose.covariance[i] = 0.0;
            filtered_odom.twist.covariance[i] = 0.0;
        }
        // 位置协方差（对角线元素）
        filtered_odom.pose.covariance[0] = 0.01;   // x
        filtered_odom.pose.covariance[7] = 0.01;   // y
        filtered_odom.pose.covariance[14] = 0.01;  // z
        filtered_odom.pose.covariance[21] = 0.05;  // roll
        filtered_odom.pose.covariance[28] = 0.05;  // pitch
        filtered_odom.pose.covariance[35] = 0.05;  // yaw
        
        // 速度协方差
        filtered_odom.twist.covariance[0] = 0.01;   // vx
        filtered_odom.twist.covariance[7] = 0.01;   // vy
        filtered_odom.twist.covariance[14] = 0.01;  // vz
        filtered_odom.twist.covariance[21] = 0.01;  // wx
        filtered_odom.twist.covariance[28] = 0.01;  // wy
        filtered_odom.twist.covariance[35] = 0.01;  // wz
        
        m_filtered_odom_pub->publish(filtered_odom);
    }

    void sendBroadCastTF(builtin_interfaces::msg::Time &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = m_config.map_frame;
        transformStamped.child_frame_id = m_config.local_frame;
        transformStamped.header.stamp = time;
        Eigen::Quaterniond q(m_state.last_offset_r);
        V3D t = m_state.last_offset_t;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        m_tf_broadcaster->sendTransform(transformStamped);
    }

    void relocCB(const std::shared_ptr<interface::srv::Relocalize::Request> request, std::shared_ptr<interface::srv::Relocalize::Response> response)
    {
        std::string pcd_path = request->pcd_path;
        float x = request->x;
        float y = request->y;
        float z = request->z;
        float yaw = request->yaw;
        float roll = request->roll;
        float pitch = request->pitch;

        if (!std::filesystem::exists(pcd_path))
        {
            response->success = false;
            response->message = "pcd file not found";
            return;
        }

        Eigen::AngleAxisd yaw_angle = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd roll_angle = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
        bool load_flag = m_localizer->loadMap(pcd_path);
        if (!load_flag)
        {
            response->success = false;
            response->message = "load map failed";
            return;
        }
        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            m_state.initial_guess.setIdentity();
            m_state.initial_guess.block<3, 3>(0, 0) = (yaw_angle * roll_angle * pitch_angle).toRotationMatrix().cast<float>();
            m_state.initial_guess.block<3, 1>(0, 3) = V3F(x, y, z);
            m_state.service_received = true;
            m_state.localize_success = false;
        }

        response->success = true;
        response->message = "relocalize success";
        return;
    }

    void relocCheckCB(const std::shared_ptr<interface::srv::IsValid::Request> request, std::shared_ptr<interface::srv::IsValid::Response> response)
    {
        std::lock_guard<std::mutex> lock(m_state.service_mutex);
        if (request->code == 1)
            response->valid = true;
        else
            response->valid = m_state.localize_success;
        return;
    }

    void publishMapCloud(builtin_interfaces::msg::Time &time)
    {
        if (m_map_cloud_pub->get_subscription_count() < 1)
            return;
        CloudType::Ptr map_cloud = m_localizer->refineMap();
        if (map_cloud->size() < 1)
            return;
        sensor_msgs::msg::PointCloud2 map_cloud_msg;
        pcl::toROSMsg(*map_cloud, map_cloud_msg);
        map_cloud_msg.header.frame_id = m_config.map_frame;
        map_cloud_msg.header.stamp = time;
        m_map_cloud_pub->publish(map_cloud_msg);
    }

private:
    NodeConfig m_config;
    NodeState m_state;

    ICPConfig m_localizer_config;
    std::shared_ptr<ICPLocalizer> m_localizer;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    rclcpp::Service<interface::srv::Relocalize>::SharedPtr m_reloc_srv;
    rclcpp::Service<interface::srv::IsValid>::SharedPtr m_reloc_check_srv;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_cloud_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_filtered_odom_pub;  // 滤波后的里程计发布器

    // 回调组
    rclcpp::CallbackGroup::SharedPtr m_timer_callback_group;
    rclcpp::CallbackGroup::SharedPtr m_subscriber_callback_group;
    rclcpp::CallbackGroup::SharedPtr m_service_callback_group;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // 创建多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    auto node = std::make_shared<LocalizerNode>();
    
    // 将节点添加到执行器
    executor.add_node(node);
    
    // 启动执行器
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}