// ROS Node to change the IMU frame from the camera frame to the IMU frame

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// Create a publisher for the new IMU message
ros::Publisher pub_imu;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{   
    // Retrieve the linear acceleration from the IMU message as a vector
    Eigen::Vector3d linear_acceleration(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

    // Create a rotation matrix with manual values
    Eigen::Matrix3d R;
    R << 0.,  0.,  1.,
        -1.,  0.,  0.,
         0., -1.,  0.;

    // Rotate the linear acceleration vector by R
    Eigen::Vector3d linear_acceleration_rotated = R * linear_acceleration;
    Eigen::Vector3d angular_velocity_rotated = R * Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    // Publish the new imu message
    sensor_msgs::Imu msg_new;
    msg_new.header = msg->header;
    msg_new.header.frame_id = "camera_accel_frame";
    msg_new.linear_acceleration.x = linear_acceleration_rotated(0);
    msg_new.linear_acceleration.y = linear_acceleration_rotated(1);
    msg_new.linear_acceleration.z = linear_acceleration_rotated(2);
    msg_new.angular_velocity.x = angular_velocity_rotated(0);
    msg_new.angular_velocity.y = angular_velocity_rotated(1);
    msg_new.angular_velocity.z = angular_velocity_rotated(2);
    msg_new.orientation = msg->orientation;
    pub_imu.publish(msg_new);

}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "change_imu_node");
    ros::NodeHandle nh;

    ros::Subscriber imu_sub = nh.subscribe("/camera/imu", 400, imuCallback);

    ros::spin();
    return 0;
}
