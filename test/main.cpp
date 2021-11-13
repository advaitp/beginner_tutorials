#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testpublisher");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}