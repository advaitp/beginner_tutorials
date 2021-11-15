/**
 * @file testpublisher.cpp
 * @brief program to run different tests related to publisher, tf and service
 * @author Advait Patole
 *
 * Copyright (c) 2021 Advait Patole
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/change_string.h"
#include <tf/transform_broadcaster.h>
#include "tf/transform_listener.h"


TEST(TESTSuite, checkService)
{
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<beginner_tutorials::change_string>("change_string");
  bool exists(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(exists);

  beginner_tutorials::change_string srv;
  srv.request.input = "Test for service call";
  client.call(srv);
  EXPECT_EQ(srv.response.output, "Test for service call");
}

TEST(TESTSuite1, checktfbroadcast)
{
  ros::NodeHandle nh;
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



