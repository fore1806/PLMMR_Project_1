#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "tf/transform_broadcaster.h"
#include <cmath>

class Odometer{
    private:
        // ROS NodeHandle
        ros:: NodeHandle n;

        // ROS Publisher and Subscriber
        ros::Publisher odom_pub;
        ros::Subscriber odom_sub;
        ros::Publisher path_pub;

        // ROS Transform Broadcaster
        tf::TransformBroadcaster odom_br;
        tf::Transform transform; // Transform object (message)
        tf::Quaternion q; // Quaternion object (message)

        // Timer Values
        ros::Time current_time, last_time;
        
        // Constants

        double d; // Distance between rear and front wheel [m]
        double L; // Rear wheel baseline [m]
        double b; // Half of the rear wheel baseline [m]
        double STEERING_FACTOR; // Steering factor

        double x; // x position
        double y; // y position
        double theta; // Orientation
        double delta_time; // Delta time [s]
        double R; // Radius of curvature [m]
        double omega; // Angular velocity [rad/s]

        // Subscriber variables
        double speed;
        double speed_m_s;
        double steer_angle;
        double wheel_angle;

        nav_msgs::Path path_msg; // Oggetto path

    public: 
        
        Odometer(){
            // Initialize the position and orientation
            x = 0.0;
            y = 0.0;
            theta = M_PI/2.0; // Start facing north

            // Set the vehicles parameters
            if(n.getParam("d", d)){
                ROS_INFO("Distance between rear and front wheel: %f", d);
            } else{
                ROS_WARN("Distance between rear and front wheel not set, using default value: %f", d);
            }

            if(n.getParam("L", L)){
                ROS_INFO("Rear wheel baseline: %f", L);
            } else{
                ROS_WARN("Rear wheel baseline not set, using default value: %f", L);
            }

            if(n.getParam("STEERING_FACTOR", STEERING_FACTOR)){
                ROS_INFO("Steering factor: %f", STEERING_FACTOR);
            } else{
                ROS_WARN("Steering factor not set, using default value: %f", STEERING_FACTOR);
            }
            b = L/2.0; // Half of the rear wheel baseline [m]

            // Initialize the time
            last_time = ros::Time::now();
            current_time = ros::Time::now();

            // Initialize the ROS Subscriber
            odom_sub = n.subscribe("/speedsteer", 1000, &Odometer::speedSteerCallback, this);

            // Initialize the ROS Publisher
            odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 1000);

            // Initialize the ROS Path Publisher
            path_pub = n.advertise<nav_msgs::Path>("/path", 1000);
            path_msg.header.frame_id = "world";
        }

        void ackerman(){            // Calculate the new position and orientation using the Ackermann model and bycicle model
            // R = d/std::tan(wheel_angle); // Radius of curvature
            omega = speed_m_s*std::tan(wheel_angle)/d; // Angular velocity

            // Calculate delta time
            current_time = ros::Time::now();
            delta_time = (current_time - last_time).toSec();
            last_time = current_time;

            // Update the position and orientation
            x += speed_m_s*std::cos(theta)*delta_time; // Update x position
            y += speed_m_s*std::sin(theta)*delta_time; // Update y position
            theta += omega*delta_time; // Update orientation
            theta = std::fmod(theta + M_PI, 2*M_PI) - M_PI; // Normalize theta to [-pi, pi]
        }

        void publishOdometry() {
            // Create a transform object
            transform.setOrigin(tf::Vector3(x, y, 0.0)); // Set the position
            // Update the quaternion based on the orientation
            q.setRPY(0, 0, theta); // Set the orientation
            transform.setRotation(q); // Set the rotation

            // Send the transform
            odom_br.sendTransform(tf::StampedTransform(transform, current_time, "world", "odom-vehicle"));

            // Create a new Odometry message
            nav_msgs::Odometry odom;

            // Set the header information
            odom.header.stamp = current_time; // Set the timestamp
            odom.header.frame_id = "world"; // Set the frame ID
            odom.child_frame_id = "odom-vehicle"; // Set the child frame ID

            // Set the position
            odom.pose.pose.position.x = x; // Set the x position
            odom.pose.pose.position.y = y; // Set the y position
            odom.pose.pose.position.z = 0.0; // Set the z position

            // Set the orientation using the quaternion
            odom.pose.pose.orientation.x = q.x(); // Set the orientation x
            odom.pose.pose.orientation.y = q.y(); // Set the orientation y
            odom.pose.pose.orientation.z = q.z(); // Set the orientation z
            odom.pose.pose.orientation.w = q.w(); // Set the orientation w

            // Set the speed and angular velocity
            odom.twist.twist.linear.x = speed_m_s; // Set the linear velocity in x direction
            odom.twist.twist.linear.y = 0.0; // Set the linear velocity in y direction
            odom.twist.twist.angular.z = omega; // Set the angular velocity

            // Set the covariance pose matrix
            for (int i = 0; i < 36; ++i) {
                odom.pose.covariance[i] = 0.0;
            }

            odom.pose.covariance[0] = 0.1;  // x
            odom.pose.covariance[7] = 0.1;  // y
            odom.pose.covariance[35] = 0.2;  // yaw

            // Set the covariance twist matrix
            for (int i = 0; i < 36; ++i) {
                odom.twist.covariance[i] = 0.0;
            }

            // Set the covariance values for linear and angular velocities (Reference values available on the web)
            odom.twist.covariance[0] = 0.1;  // vx
            odom.twist.covariance[7] = 0.1;  // vy
            odom.twist.covariance[35] = 0.2;  // yaw rate

            // Publish the odometry message
            odom_pub.publish(odom); // Publish the odometry message
        }

        void publishPath() {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = current_time;
            pose_stamped.header.frame_id = "world";
            pose_stamped.pose.position.x = x;
            pose_stamped.pose.position.y = y;
            pose_stamped.pose.position.z = 0.0;
            pose_stamped.pose.orientation.x = q.x();
            pose_stamped.pose.orientation.y = q.y();
            pose_stamped.pose.orientation.z = q.z();
            pose_stamped.pose.orientation.w = q.w();

            path_msg.poses.push_back(pose_stamped);
            path_msg.header.stamp = current_time;
            path_pub.publish(path_msg);
        }

        void speedSteerCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
        {
            // Process the speed and steering angle data
            speed = msg->point.y; // Speed in km/h
            speed_m_s = speed/3.6; // Convert speed from km/h to m/s
            steer_angle = msg->point.x; // Steering wheel angle in degrees

            //Smoothing Filter
            if (abs(steer_angle)<9.0){
                steer_angle = 0;
            }
            
            wheel_angle = steer_angle*(M_PI/180.0)/ STEERING_FACTOR; 
        
            // Print the received data
            ROS_INFO("Speed: %f, Wheel Angle: %f, Steering Angle: %f", speed, wheel_angle, steer_angle);

            // Call the Ackermann function to update the position and orientation
            ackerman();
            // Print the updated position and orientation
            ROS_INFO("Updated Position: x = %f, y = %f, theta = %f", x, y, theta);
            
            // Publish the odometry
            publishOdometry();

            // Publish the path in Rviz
            publishPath();
        }
};


int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "odometer_node");

    // Create an instance of the Odometer class
    Odometer odometer;

    // Spin to keep the node running
    ros::spin();

    return 0;
}