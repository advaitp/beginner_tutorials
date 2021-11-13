#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/change_string.h"
#include <tf/transform_broadcaster.h>
#include "tf/transform_listener.h"

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

TEST(TESTSuite1, checktfbroadcast)
{
  tf::StampedTransform transform;
  tf::TransformListener listener;
  
  while (ros::ok()) {
    try {
      listener.lookupTransform("talk", "world", ros::Time(0), transform);
      break;
    } catch (tf::TransformException &ex) {
      ROS_ERROR(ex.what());
      continue;
    }
  }

  int x_coord, y_coord, z_coord ;
  x_coord = transform.getOrigin().x();
  y_coord = transform.getOrigin().y();
  z_coord = transform.getOrigin().z();
  EXPECT_TRUE((x_coord >= -20) && (x_coord <= 20));
  EXPECT_TRUE((y_coord >= -20) && (y_coord <= 20));
  EXPECT_TRUE((z_coord >= -20) && (z_coord <= 20));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testpublisher");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

