#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>

class MapOverlay : public rclcpp::Node
{
public:
    MapOverlay() : Node("map_overlay_node")
    {
        // Load the ground truth map (assumes it is stored locally)
        ground_truth_map_ = cv::imread("/home/student/ros2_ws/src/package/src/ground_truth_map.pgm", cv::IMREAD_GRAYSCALE);
        if (ground_truth_map_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not load the ground truth map.");
            rclcpp::shutdown();
        }

        // Find the bounding box of the room in the ground truth map
        cv::threshold(ground_truth_map_, binary_map_, 200, 255, cv::THRESH_BINARY_INV);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary_map_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Get the bounding rectangle of the largest contour (assuming it is the room footprint)
        cv::Rect room_roi = cv::boundingRect(contours[0]);
        for (size_t i = 1; i < contours.size(); i++)
        {
            cv::Rect current_rect = cv::boundingRect(contours[i]);
            room_roi = room_roi | current_rect; // Expand to include all detected contours
        }

        // Crop the ground truth map to get only the actual room footprint
        ground_truth_map_cropped_ = ground_truth_map_(room_roi).clone();
        cv::cvtColor(ground_truth_map_cropped_, ground_truth_map_cropped_, cv::COLOR_GRAY2BGR); // Convert to color for overlay

        // Subscription to SLAM map topic
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MapOverlay::map_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    cv::Mat ground_truth_map_, ground_truth_map_cropped_, binary_map_;

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        // Extract information from OccupancyGrid
        const auto &map_data = msg->data;
        int width = msg->info.width;
        int height = msg->info.height;

        // Create a new image from the SLAM map data
        cv::Mat slam_map(height, width, CV_8UC1, cv::Scalar(205)); // Initialize as unknown (light gray)

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int index = y * width + x;
                if (map_data[index] == 0)
                {
                    slam_map.at<uchar>(y, x) = 255; // Free space (white)
                }
                else if (map_data[index] == 100)
                {
                    slam_map.at<uchar>(y, x) = 0; // Occupied space (black)
                }
                else if (map_data[index] == -1)
                {
                    slam_map.at<uchar>(y, x) = 205; // Unknown space (light gray)
                }
            }
        }

        // Flip the SLAM map vertically to align with the ground truth map
        cv::flip(slam_map, slam_map, 0);

        // Resize SLAM map to match the size of the cropped ground truth map for consistent visualization
        cv::resize(slam_map, slam_map, ground_truth_map_cropped_.size());

        // Convert SLAM map to BGR color for better visualization and overlay
        cv::Mat slam_map_color;
        cv::cvtColor(slam_map, slam_map_color, cv::COLOR_GRAY2BGR);

        // Apply color to the SLAM map to differentiate from the ground truth map
        for (int y = 0; y < slam_map_color.rows; y++)
        {
            for (int x = 0; x < slam_map_color.cols; x++)
            {
                if (slam_map.at<uchar>(y, x) == 0)
                {
                    slam_map_color.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255); // Occupied space in red
                }
                else if (slam_map.at<uchar>(y, x) == 255)
                {
                    slam_map_color.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255); // Free space in white
                }
                else if (slam_map.at<uchar>(y, x) == 205)
                {
                    slam_map_color.at<cv::Vec3b>(y, x) = cv::Vec3b(200, 200, 200); // Unknown space in light gray
                }
            }
        }

        // Overlay the SLAM map onto the cropped ground truth map
        cv::Mat overlay;
        cv::addWeighted(ground_truth_map_cropped_, 0.4, slam_map_color, 0.6, 0.0, overlay);

        // Display the overlay
        cv::imshow("SLAM Map Overlay", overlay);

        // Display the SLAM map separately
        cv::imshow("SLAM Map Only", slam_map_color);

        cv::waitKey(1);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapOverlay>());
    rclcpp::shutdown();
    return 0;
}
