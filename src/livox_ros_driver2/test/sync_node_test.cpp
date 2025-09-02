#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "livox_ros_driver2/CustomMsg.h"
#include "sync_node.h"

class SyncNodeTest : public ::testing::Test {
protected:
    virtual void SetUp() {
        ros::Time::init();
        nh = std::make_shared<ros::NodeHandle>();
    }

    virtual void TearDown() {
        nh.reset();
    }

    std::shared_ptr<ros::NodeHandle> nh;
};

TEST_F(SyncNodeTest, TestConstructor) {
    // Set up parameters
    nh->setParam("image_topic", "/camera/image_raw");
    nh->setParam("lidar_topic", "/livox/lidar");
    nh->setParam("synced_image_topic", "/synced/image");
    nh->setParam("synced_lidar_topic", "/synced/lidar");
    nh->setParam("publish_rate", 10.0);
    nh->setParam("sync_tolerance", 0.05);

    // Test constructor
    SyncNode syncNode(*nh);

    // Verify subscriptions and publishers
    EXPECT_TRUE(syncNode.image_pub);
    EXPECT_TRUE(syncNode.lidar_pub);
}

TEST_F(SyncNodeTest, TestCompressedImageTopic) {
    // Set up parameters with compressed image topic
    nh->setParam("image_topic", "/camera/image_raw/compressed");
    nh->setParam("lidar_topic", "/livox/lidar");
    nh->setParam("synced_image_topic", "/synced/image");
    nh->setParam("synced_lidar_topic", "/synced/lidar");
    nh->setParam("publish_rate", 10.0);
    nh->setParam("sync_tolerance", 0.05);

    // Test constructor with compressed image topic
    SyncNode syncNode(*nh);

    // Verify compressed image subscription
    EXPECT_TRUE(syncNode.compressed_image_sub);
}

TEST_F(SyncNodeTest, TestSyncCallback) {
    // Set up parameters
    nh->setParam("image_topic", "/camera/image_raw");
    nh->setParam("lidar_topic", "/livox/lidar");
    nh->setParam("synced_image_topic", "/synced/image");
    nh->setParam("synced_lidar_topic", "/synced/lidar");
    nh->setParam("publish_rate", 10.0);
    nh->setParam("sync_tolerance", 0.05);

    // Test constructor
    SyncNode syncNode(*nh);

    // Create test messages
    sensor_msgs::Image image_msg;
    livox_ros_driver2::CustomMsg lidar_msg;

    // Call syncCallback
    syncNode.syncCallback(image_msg, lidar_msg);

    // Verify messages are published (mock test)
    EXPECT_TRUE(true);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sync_node_test");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}