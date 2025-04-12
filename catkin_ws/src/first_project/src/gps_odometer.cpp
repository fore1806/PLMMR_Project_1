#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "tf/transform_broadcaster.h"
#include <cmath>


struct ECEF{
    double x_ECEF; // ECEF x coordinate
    double y_ECEF; // ECEF y coordinate
    double z_ECEF; // ECEF z coordinate
};

ECEF transform_ECEF(double lat_val, double lon_val, double alt_val)
{   
    // Constants
    const double a = 6378137.0; // Earth's radius [m]
    const double b = 6356752.0; // Earth's semi-minor axis [m]
    const double e_square = 1 - (b*b)/(a*a); // Square of eccentricity
    
    // Convert latitude and longitude from degrees to radians
    double lat_rad = lat_val * M_PI / 180.0;
    double lon_rad = lon_val * M_PI / 180.0;

    // Calculate the ECEF coordinates
    double N = a / sqrt(1 - e_square * sin(lat_rad) * sin(lat_rad));
    double x_ECEF = (N + alt_val) * cos(lat_rad) * cos(lon_rad);
    double y_ECEF = (N + alt_val) * cos(lat_rad) * sin(lon_rad);
    double z_ECEF = ((1 - e_square) * N + alt_val) * sin(lat_rad);

    return {x_ECEF, y_ECEF, z_ECEF};
}

class GPSOdometer {
    private:
        // ROS NodeHandle
        ros:: NodeHandle n;

        // ROS Publisher and Subscriber
        ros::Publisher gps_odom_pub;
        ros::Subscriber gps_odom_sub;
        ros::Publisher path_pub_gps;

        // ROS Transform Broadcaster
        tf::TransformBroadcaster gps_odom_br;
        tf::Transform transform; // Transform object (message)
        tf::Quaternion q; // Quaternion object (message)

        // Timer Values
        ros::Time current_time, last_time;
        
        // Constants       
        double x; // x position
        double y; // y position
        double z; // z position
        double theta; // Orientation
        double x_prev; // Previous x position
        double y_prev; // Previous y position
        double theta_prev; // Previous orientation
        double delta_time; // Delta time [s]
        
        // Subscriber variables
        double lat; // Latitude in degrees
        double lon; // Longitude in degrees
        double alt; // Altitude in meters

        // Reference Data
        const double lat_r = 45.618932386592405; // Reference latitude in grades
        const double lon_r = 9.281178887031235; // Reference longitude in grades
        const double alt_r = 229.04906147731415; // Reference altitude in meters

        ECEF reference_ECEF; // Reference ECEF coordinates
        double X_r;// = reference_ECEF.x_ECEF; // Reference ECEF x coordinate
        double Y_r;// = reference_ECEF.y_ECEF; // Reference ECEF y coordinate
        double Z_r;// = reference_ECEF.z_ECEF; // Reference ECEF z coordinate

        nav_msgs::Path path_msg; // Oggetto path

    public: 
        
        GPSOdometer(){
            // Initialize the position and orientation
            x = 0.0;
            y = 0.0;
            z = 0.0;
            x_prev = 0.0;
            y_prev = 0.0;

            theta = 0.0;
            theta_prev = 0.0;

            reference_ECEF = transform_ECEF(lat_r, lon_r, alt_r); // Reference ECEF coordinates
            X_r = reference_ECEF.x_ECEF; // Reference ECEF x coordinate
            Y_r = reference_ECEF.y_ECEF; // Reference ECEF y coordinate
            Z_r = reference_ECEF.z_ECEF; // Reference ECEF z coordinate


            // Initialize the time
            last_time = ros::Time::now();
            current_time = ros::Time::now();

            // Initialize the ROS Subscriber
            gps_odom_sub = n.subscribe("/swiftnav/front/gps_pose", 1000, &GPSOdometer::gpsCallback, this);

            // Initialize the ROS Publisher
            gps_odom_pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1000);

            // Initialize the ROS Path Publisher
            path_pub_gps = n.advertise<nav_msgs::Path>("/gps_path", 1000);
            path_msg.header.frame_id = "world";
        }

        void ECEF_Odometry(){
            ECEF current_ECEF = transform_ECEF(lat, lon, alt); // Transform the GPS coordinates to ECEF
            double X = current_ECEF.x_ECEF; // ECEF x coordinate
            double Y = current_ECEF.y_ECEF; // ECEF y coordinate
            double Z = current_ECEF.z_ECEF; // ECEF z coordinate

            double phi_r = lat_r * M_PI / 180.0; // Reference latitude in radians
            double lambda_r = lon_r * M_PI / 180.0; // Reference longitude in radians
            
            // Calculate the position and orientation
            x = -(X-X_r)*sin(lambda_r) + (Y-Y_r)*cos(lambda_r); // Calculate the x position
            y = -(X-X_r)*sin(phi_r)*cos(lambda_r) - (Y-Y_r)*sin(phi_r)*sin(lambda_r) + (Z-Z_r)*cos(phi_r); // Calculate the y position
            z = (X-X_r)*cos(phi_r)*cos(lambda_r) + (Y-Y_r)*cos(phi_r)*sin(lambda_r) + (Z-Z_r)*sin(phi_r); // Calculate the z position
            theta = atan2(y - y_prev, x - x_prev); // Calculate the orientation

            // Update the reference ECEF coordinates
            // X_r = X; // Update the reference ECEF x coordinate
            // Y_r = Y; // Update the reference ECEF y coordinate
            // Z_r = Z; // Update the reference ECEF z coordinate
        }

        void publishPose() {
            // Create a transform object
            transform.setOrigin(tf::Vector3(x, y, 0.0)); // Set the position
            // Update the quaternion based on the orientation
            q.setRPY(0, 0, theta); // Set the orientation
            transform.setRotation(q); // Set the rotation

            // Send the transform
            gps_odom_br.sendTransform(tf::StampedTransform(transform, current_time, "world", "odom-gps"));

            // Create a new Odometry message
            //next, we'll publish the odometry message over ROS
            nav_msgs::Odometry gps_odom;

            // Set the header information
            gps_odom.header.stamp = current_time; // Set the timestamp
            gps_odom.header.frame_id = "world"; // Set the frame ID
            gps_odom.child_frame_id = "odom-gps"; // Set the child frame ID

            // Set the position
            gps_odom.pose.pose.position.x = x; // Set the x position
            gps_odom.pose.pose.position.y = y; // Set the y position
            gps_odom.pose.pose.position.z = 0.0; // Set the z position

            // Set the orientation using the quaternion
            gps_odom.pose.pose.orientation.x = q.x(); // Set the orientation x
            gps_odom.pose.pose.orientation.y = q.y(); // Set the orientation y
            gps_odom.pose.pose.orientation.w = q.w(); // Set the orientation w
            gps_odom.pose.pose.orientation.z = q.z(); // Set the orientation z


            // Calculate the delta time
            current_time = ros::Time::now(); // Get the current time
            delta_time = (current_time - last_time).toSec(); // Calculate the delta time
            last_time = current_time; // Update the last time

            // Set the velocita in the odometry message
            gps_odom.twist.twist.linear.x = std::sqrt(std::pow(x-x_prev,2) + std::pow(y-y_prev,2))/delta_time; // Set the linear velocity in x direction
            gps_odom.twist.twist.linear.y = 0; // Set the linear velocity in y direction
            gps_odom.twist.twist.angular.z = (theta - theta_prev)/delta_time; // Set the angular velocity

            // Update the previous position and orientation
            x_prev = x; // Update the previous x position
            y_prev = y; // Update the previous y position
            theta_prev = theta; // Update the previous orientation


            // Publish the odometry message
            gps_odom_pub.publish(gps_odom); // Publish the odometry message
        }

        void publishPath() {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = current_time;
            pose_stamped.header.frame_id = "world";
            pose_stamped.pose.position.x = x;
            pose_stamped.pose.position.y = y;
            pose_stamped.pose.position.z = z;

            pose_stamped.pose.orientation.x = q.x();
            pose_stamped.pose.orientation.y = q.y();
            pose_stamped.pose.orientation.z = q.z();
            pose_stamped.pose.orientation.w = q.w();

            path_msg.poses.push_back(pose_stamped);
            path_msg.header.stamp = current_time;
            path_pub_gps.publish(path_msg);
        }

        void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
        {
            // Process the speed and steering angle data
            lat = msg->latitude; // Latitude in degrees
            lon = msg->longitude; // Longitude in degrees
            alt = msg->altitude; // Altitude in meters
        
            // Print the received data
            ROS_INFO("Latitude: %f, Longitude: %f, Altitude: %f", lat, lon, alt);

            // Call the Ackermann function to update the position and orientation
            ECEF_Odometry();

            // Print the updated position and orientation
            ROS_INFO("Updated Position: x = %f, y = %f, theta = %f", x, y, theta);
            
            // Publish the odometry
            publishPose();

            // Publish the path in Rviz
            publishPath();
        }
};

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "gps_odometer_node");

    // Create an instance of the gps Odometer class
    GPSOdometer gps_odometer;

    // Spin to keep the node running
    ros::spin();

    return 0;
}