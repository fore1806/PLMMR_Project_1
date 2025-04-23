#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PointStamped.h"
#include "first_project/secotor_times.h"
#include <cmath>

struct Sector{
    int sector_number; // Sector number
    float time; // Time in sector
    float mean_speed; // Speed in sector
    float total_speed_sum; // Total speed sum in sector
    unsigned int n; // Number of time stamps 
};

class Sector_Times {

    private:
        // ROS NodeHandle
        ros::NodeHandle n;

        // ROS Subscriber
        ros::Subscriber gps_sub;
        ros::Subscriber speed_sub;

        // ROS Publisher
        ros::Publisher sector_times_pub;

        // Timer Values
        ros::Time current_time, last_time;

        // Delta_time
        double delta_time = 0.0;

        // Sectors Position
        const double SECTOR_1_LAT = 45.63011; // LAtitude Sector 1
        const double SECTOR_1_LON = 9.28949; // Longitude Sector 1

        const double SECTOR_2_LAT = 45.62354; // Latitude Sector 2
        const double SECTOR_2_LON = 9.28725; // Longitude Sector 2

        const double SECTOR_3_LAT = 45.61606; // Latitude Sector 3
        const double SECTOR_3_LON = 9.28076; // Longitude Sector 3

        // Declaration of the sectors
        Sector sector_1; // Sector 1 info
        Sector sector_2; // Sector 2 info
        Sector sector_3; // Sector 3 info
        Sector actual_sector; // Current sector info

        // Current Sector number
        int current_sector_number; // Current sector number

        // GPS first values
        double lat0; // Latitude
        double lon0; // Longitude
        double alt0; // Altitude

        bool isFirst = true;
        
    public:

        Sector_Times() {
            // Initialize the sectors
            sector_1.sector_number = 1;
            sector_1.time = 0.0;
            sector_1.mean_speed = 0.0;
            sector_1.total_speed_sum = 0.0;
            sector_1.n = 0;

            sector_2.sector_number = 2;
            sector_2.time = 0.0;
            sector_2.mean_speed = 0.0;
            sector_2.total_speed_sum = 0.0;
            sector_2.n = 0;

            sector_3.sector_number = 3;
            sector_3.time = 0.0;
            sector_3.mean_speed = 0.0;
            sector_3.total_speed_sum = 0.0;
            sector_3.n = 0;

            // Get the reference GPS coordinates from the parameter server
            if(n.getParam("/lat_r", lat0)){
                ROS_INFO("Reference latitude: %.15f", lat0);
            } else {
                ROS_WARN("Failed to get param 'lat_r'");
            }
        
            if(n.getParam("/lon_r", lon0)){
                ROS_INFO("Reference longitude: %.15f", lon0);
            } else {
                ROS_WARN("Failed to get param 'lon_r'");
            }
        
            if(n.getParam("/alt_r", alt0)){
                ROS_INFO("Reference altitude: %.15f", alt0);
            } else {
                ROS_WARN("Failed to get param 'alt_r'");
            }

            // Initialize the current sector
            actual_sector = sector_1; // Start in sector 1
            current_sector_number = 1; // Start in sector 1

            // Initialize the time
            last_time = ros::Time::now();
            current_time = ros::Time::now();

            // Initialize the ROS Subscriber
            gps_sub = n.subscribe("/swiftnav/front/gps_pose", 1000, &Sector_Times::gpsCallback, this);

            // Initialize the ROS Subscriber
            speed_sub = n.subscribe("/speedsteer", 1000, &Sector_Times::speedCallback, this);
            
            // Initialize the ROS Publisher
            sector_times_pub = n.advertise<first_project::secotor_times>("/sector_times", 1000);
        }

        //Sector currentSector(double gps_lat, double gps_lon) {
        //    // Check if the vehicle is in sector 1
        //    if (sectorSurpassed(gps_lat, gps_lon, SECTOR_1_LAT, SECTOR_1_LON)) {
        //        return sector_2;
        //    }
        //    // Check if the vehicle is in sector 2
        //    else if (sectorSurpassed(gps_lat, gps_lon, SECTOR_2_LAT, SECTOR_2_LON)) {
        //        return sector_3;
        //    }
        //    // Check if the vehicle is in sector 3
        //    else if (sectorSurpassed(gps_lat, gps_lon, SECTOR_3_LAT, SECTOR_3_LON)) {
        //        return sector_1;
        //    } else{
        //        return actual_sector; // The actual sector has not changed
        //    }
        //                
        //}

        int currentSector(double gps_lat, double gps_lon) {
            // Check if the vehicle is in sector 1
            if (sectorSurpassed(gps_lat, gps_lon, SECTOR_1_LAT, SECTOR_1_LON)) {
                return 2;
            }
            // Check if the vehicle is in sector 2
            else if (sectorSurpassed(gps_lat, gps_lon, SECTOR_2_LAT, SECTOR_2_LON)) {
                return 3;
            }
            // Check if the vehicle is in sector 3
            else if (sectorSurpassed(gps_lat, gps_lon, SECTOR_3_LAT, SECTOR_3_LON)) {
                return 1;
            } else{
                return current_sector_number; // The actual sector has not changed
            }
                        
        }

        bool sectorSurpassed(double gps_lat, double gps_lon, double sector_lat, double sector_lon) {
            // Check if the vehicle is in the sector
            if (gps_lat >= sector_lat - 0.0001 && gps_lat <= sector_lat + 0.0001 &&
                gps_lon >= sector_lon - 0.0001 && gps_lon <= sector_lon + 0.0001) {
                return true; // In the sector
            }
            else {
                return false; // Not in the sector
            }
        }

        void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
            // Process the GPS data
            double lat = msg->latitude; // Latitude in degrees
            double lon = msg->longitude; // Longitude in degrees
            double alt = msg->altitude; // Altitude in meters

            // Reset the time when the first value is received
            if (isFirst) {
                last_time = ros::Time::now(); // Reset the last time
                delta_time = 0.0;
                ROS_INFO("lat: %.15f", lat);
                ROS_INFO("lon: %.15f", lon);
                ROS_INFO("alt: %.15f", alt);
                isFirst = false;
            }

            // Calculate the delta time
            current_time = ros::Time::now(); // Get the current time
            delta_time = (current_time - last_time).toSec(); // Calculate the delta time
            last_time = current_time; // Update the last time

            // We find in which sector the vehicle is
            current_sector_number = currentSector(lat, lon); // Get the current sector

            // We update the values
            switch (current_sector_number) {
                case 1:
                    sector_1.time += delta_time; // Update the time in the actual sector
                    break;
                case 2:
                    sector_2.time += delta_time; // Update the time in the actual sector
                    break;
                case 3:
                    sector_3.time += delta_time; // Update the time in the actual sector
                    break;
                default:
                    break;
            }

            if (lat == 45.61891905788306 && lon == 9.281144863247937){
                ROS_INFO("È FINITO");
                ROS_INFO("Sector 1 time: %f, Sector 2 time: %f, Sector 3 time: %f", sector_1.time, sector_2.time, sector_3.time);
                ROS_INFO("Mean Speed Sector 1: %f, Mean Speed Sector 2: %f, Mean Speed Sector 3: %f", sector_1.mean_speed, sector_2.mean_speed, sector_3.mean_speed);

            }
            

            publishSectorTimes();
        }

        void speedCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
            // Process the speed data
            double speed = msg->point.y; // Speed in km/h

            switch (current_sector_number) {
                case 1:
                    sector_1.total_speed_sum += speed;
                    sector_1.n += 1;
                    sector_1.mean_speed = sector_1.total_speed_sum/sector_1.n;
                    break;
                case 2:
                    sector_2.total_speed_sum += speed;
                    sector_2.n += 1;
                    sector_2.mean_speed = sector_2.total_speed_sum/sector_2.n;
                    break;
                case 3:
                    sector_3.total_speed_sum += speed;
                    sector_3.n += 1;
                    sector_3.mean_speed = sector_3.total_speed_sum/sector_3.n;
                    break;
                default:
                    break;
            }
        }

        void publishSectorTimes() {
            // Create a new sector times message
            first_project::secotor_times sector_times_msg;

            sector_times_msg.current_sector = current_sector_number; // Current sector number

            switch (current_sector_number) {
                case 1:
                    sector_times_msg.current_sector_time = sector_1.time;
                    sector_times_msg.current_sector_mean_speed = sector_1.mean_speed;
                    break;
                case 2:
                    sector_times_msg.current_sector_time = sector_2.time;
                    sector_times_msg.current_sector_mean_speed = sector_2.mean_speed;
                    break;
                case 3:
                    sector_times_msg.current_sector_time = sector_3.time;
                    sector_times_msg.current_sector_mean_speed = sector_3.mean_speed;
                    break;
                default:
                    break;
            }
            // What are we going to publish?
            //ROS_INFO("Sector: %d, Time: %f, Mean Speed: %f", sector_times_msg.current_sector, sector_times_msg.current_sector_time, sector_times_msg.current_sector_mean_speed);

            // Publish the sector times message
            sector_times_pub.publish(sector_times_msg);
        }
};

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "sector_times_node");

    // Create an instance of the Sector_Times class
    Sector_Times sector_times;

    // Spin the node
    ros::spin();

    return 0;

}