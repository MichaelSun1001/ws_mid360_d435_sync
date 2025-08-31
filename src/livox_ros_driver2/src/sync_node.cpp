#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace sensor_msgs;
using namespace livox_ros_driver2;
using namespace message_filters;

class SyncNode
{
public:
    SyncNode(ros::NodeHandle &nh)
    {
        // Get parameters from the parameter server
        std::string image_topic, lidar_topic, synced_image_topic, synced_lidar_topic;
        double publish_rate;

        nh.getParam("image_topic", image_topic);
        nh.getParam("lidar_topic", lidar_topic);
        nh.getParam("synced_image_topic", synced_image_topic);
        nh.getParam("synced_lidar_topic", synced_lidar_topic);
        nh.getParam("publish_rate", publish_rate);

        // Check if the image topic is compressed
        is_compressed = (image_topic.find("/compressed") != std::string::npos);

        if (is_compressed)
        {
            // Subscribe to compressed image
            compressed_image_sub = nh.subscribe(image_topic, 10, &SyncNode::compressedImageCallback, this);
        }
        else
        {
            // Subscribe to raw image
            raw_image_sub = nh.subscribe(image_topic, 10, &SyncNode::rawImageCallback, this);
        }

        lidar_sub = nh.subscribe(lidar_topic, 10, &SyncNode::lidarCallback, this);

        image_pub = nh.advertise<Image>(synced_image_topic, 10);
        lidar_pub = nh.advertise<CustomMsg>(synced_lidar_topic, 10);

        // Set up a timer to publish synchronized messages at the specified rate
        ros::Duration period(1.0 / publish_rate); // Convert Hz to seconds
        timer = nh.createTimer(period, &SyncNode::timerCallback, this);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber raw_image_sub;
    ros::Subscriber compressed_image_sub;
    ros::Subscriber lidar_sub;
    ros::Publisher image_pub;
    ros::Publisher lidar_pub;
    ros::Timer timer;
    bool is_compressed;

    Image last_image_msg;
    CustomMsg last_lidar_msg;
    bool new_image_received = false;
    bool new_lidar_received = false;

    void rawImageCallback(const ImageConstPtr &img_msg)
    {
        last_image_msg = *img_msg;
        new_image_received = true;
    }

    void compressedImageCallback(const CompressedImageConstPtr &compressed_img_msg)
    {
        try
        {
            // Convert compressed image to raw image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(compressed_img_msg, sensor_msgs::image_encodings::BGR8);

            // Convert back to Image message
            last_image_msg = *(cv_ptr->toImageMsg());
            last_image_msg.header = compressed_img_msg->header; // Keep original header
            new_image_received = true;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void lidarCallback(const CustomMsgConstPtr &lidar_msg)
    {
        last_lidar_msg = *lidar_msg;
        new_lidar_received = true;
    }

    void timerCallback(const ros::TimerEvent &event)
    {
        if (new_image_received && new_lidar_received)
        {
            image_pub.publish(last_image_msg);
            lidar_pub.publish(last_lidar_msg);
            new_image_received = false;
            new_lidar_received = false;
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sync_node");
    ros::NodeHandle nh("~");

    SyncNode node(nh);

    ros::spin();

    return 0;
}
