/**
 * @file publisher.cpp
 * @brief tutorial demonstrates simple sending of messages over the ROS system.
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

#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/change_string.h"

// Default string that will be displayed
extern std::string new_str = "Changed output string";

/**
 * @brief  Function to change output string using by calling rosservice
 *
 * @pre
 * @post
 * @param req
 * @param resp
 * @return true
 */
bool change(const beginner_tutorials::change_string::Request &req, /
  const beginner_tutorials::change_string::Response &res) {
  res.output = req.input;
  new_str = res.output;
  ROS_INFO("String has been modified to %s", res.output.c_str());
  return true;
}

/**
 * @brief  Main function of publisher.cpp
 *
 * @pre
 * @post
 * @param argc
 * @param argv
 * @return 0
 */
int main(int argc, char **argv)  {
  // taking input loop rate
  int rate;
  rate = atoll(argv[1]);

  if (rate < 1)  {
    ROS_FATAL_STREAM("The loop rate cannot be less than or equal to zero");
    return 0;
  } else  {
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "talker");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    /**
     * The advertise() function is how you tell ROS that you want to
     * publish on a given topic name. This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing. After this advertise() call is made, the master
     * node will notify anyone who is trying to subscribe to this topic name,
     * and they will in turn negotiate a peer-to-peer connection with this
     * node.  advertise() returns a Publisher object which allows you to
     * publish messages on that topic through a call to publish().  Once
     * all copies of the returned Publisher object are destroyed, the topic
     * will be automatically unadvertised.
     *
     * The second parameter to advertise() is the size of the message queue
     * used for publishing messages.  If messages are published more quickly
     * than we can send them, the number here specifies how many messages to
     * buffer up before throwing some away.
     */

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(rate);

    // Added the service to change the output string
    ros::ServiceServer service = n.advertiseService("change_string", change);

    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */
    int count = 0;
    while (ros::ok()) {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        std_msgs::String msg;

        // Checking the loop rate to be considered slow or normal
        if (rate < 10)  {
          ROS_WARN_STREAM("Loop rate is slow increase the loop rate");
        } else  {
          ROS_INFO_STREAM("Loop Rate is "<< rate);
        }


        std::stringstream ss;
        ss << new_str << " " <<count;
        // Checking whether some string is passed or not
        if (new_str.size() == 0)  {
          ROS_ERROR_STREAM("Send some data between nodes");
        } else {
          ROS_DEBUG_STREAM("New String is "<< new_str);
        }

        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }
  }

  return 0;
}


