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
#include <deque>

using namespace sensor_msgs;
using namespace livox_ros_driver2;
using namespace message_filters;

class SyncNode
{
public:
    SyncNode(ros::NodeHandle &nh) : sync(MySyncPolicy(10), image_sub, lidar_sub),
                                    compressed_sync(MyCompressedSyncPolicy(10), compressed_image_sub, lidar_sub)
    {
        // Get parameters from the parameter server
        std::string image_topic, lidar_topic, synced_image_topic, synced_lidar_topic;
        double publish_rate, sync_tolerance;

        nh.getParam("image_topic", image_topic);
        nh.getParam("lidar_topic", lidar_topic);
        nh.getParam("synced_image_topic", synced_image_topic);
        nh.getParam("synced_lidar_topic", synced_lidar_topic);
        nh.getParam("publish_rate", publish_rate);
        nh.param("sync_tolerance", sync_tolerance, 0.05); // Default 50ms tolerance

        // Check if the image topic is compressed
        is_compressed = (image_topic.find("/compressed") != std::string::npos);

        if (is_compressed)
        {
            // For compressed images, use message_filters with CompressedImage
            compressed_image_sub.subscribe(nh, image_topic, 10);
            lidar_sub.subscribe(nh, lidar_topic, 10);
        }
        else
        {
            // Use message_filters for raw images
            image_sub.subscribe(nh, image_topic, 10);
            lidar_sub.subscribe(nh, lidar_topic, 10);
        }

        image_pub = nh.advertise<Image>(synced_image_topic, 10);
        lidar_pub = nh.advertise<CustomMsg>(synced_lidar_topic, 10);

        // Set synchronization tolerance for both sync policies
        sync.setAgePenalty(sync_tolerance);
        compressed_sync.setAgePenalty(sync_tolerance);

        if (is_compressed)
        {
            // Register callback for compressed image synchronization
            compressed_sync.registerCallback(boost::bind(&SyncNode::compressedSyncCallback, this, _1, _2));
        }
        else
        {
            // Register callback for raw image synchronization
            sync.registerCallback(boost::bind(&SyncNode::syncCallback, this, _1, _2));
        }

        // Set up a timer to publish synchronized messages at the specified rate
        ros::Duration period(1.0 / publish_rate); // Convert Hz to seconds
        timer = nh.createTimer(period, &SyncNode::timerCallback, this);
    }

private:
    ros::NodeHandle nh;
    Subscriber<Image> image_sub;
    Subscriber<CompressedImage> compressed_image_sub;
    Subscriber<CustomMsg> lidar_sub;
    typedef sync_policies::ApproximateTime<Image, CustomMsg> MySyncPolicy;
    typedef sync_policies::ApproximateTime<CompressedImage, CustomMsg> MyCompressedSyncPolicy;
    Synchronizer<MySyncPolicy> sync;
    Synchronizer<MyCompressedSyncPolicy> compressed_sync;
    ros::Publisher image_pub;
    ros::Publisher lidar_pub;
    ros::Timer timer;
    bool is_compressed;

    Image last_image_msg;
    CustomMsg last_lidar_msg;
    bool new_sync_received = false;

    void syncCallback(const ImageConstPtr &img_msg, const CustomMsgConstPtr &lidar_msg)
    {
        // Store the latest synchronized messages for raw images
        last_image_msg = *img_msg;
        last_lidar_msg = *lidar_msg;
        new_sync_received = true;

        ROS_DEBUG("Raw image sync - Image timestamp: %f, Lidar timestamp: %f, Time diff: %f ms",
                  img_msg->header.stamp.toSec(),
                  lidar_msg->header.stamp.toSec(),
                  fabs(img_msg->header.stamp.toSec() - lidar_msg->header.stamp.toSec()) * 1000.0);
    }

    void compressedSyncCallback(const CompressedImageConstPtr &compressed_img_msg, const CustomMsgConstPtr &lidar_msg)
    {
        try
        {
            // Convert compressed image to raw image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(compressed_img_msg, sensor_msgs::image_encodings::BGR8);

            // Convert back to Image message
            Image raw_image = *(cv_ptr->toImageMsg());
            raw_image.header = compressed_img_msg->header; // Keep original header

            // Store the synchronized messages
            last_image_msg = raw_image;
            last_lidar_msg = *lidar_msg;
            new_sync_received = true;

            ROS_DEBUG("Compressed image sync - Image timestamp: %f, Lidar timestamp: %f, Time diff: %f ms",
                      compressed_img_msg->header.stamp.toSec(),
                      lidar_msg->header.stamp.toSec(),
                      fabs(compressed_img_msg->header.stamp.toSec() - lidar_msg->header.stamp.toSec()) * 1000.0);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void timerCallback(const ros::TimerEvent &event)
    {
        if (new_sync_received)
        {
            // Publish synchronized data (works for both raw and compressed images)
            image_pub.publish(last_image_msg);
            lidar_pub.publish(last_lidar_msg);
            new_sync_received = false;
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
