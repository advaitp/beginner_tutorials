#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/change_string.h"

std::shared_ptr<ros::NodeHandle> nh;

TEST(TESTSuite, checkService)
{
  ros::ServiceClient client = nh->serviceClient<beginner_tutorials::change_string>("change_string");
  bool exists(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(exists);

  beginner_tutorials::change_string srv;
  srv.request.input = "Test for service call";
  client.call(srv);
  EXPECT_EQ(srv.response.output, "Test for service call");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testpublisher");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

