/*
 * Copyright (c) 2016-2017, Atsushi Watanabe
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <vector>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <topic_tools/shape_shifter.h>

class message_switch
{
private:
  ros::NodeHandle nh;
  std::vector<ros::Subscriber> sub_topics;
  ros::Subscriber sub_select;
  ros::Publisher pub_topic;
  double timeout;
  int interrupt_button;
  ros::Time last_select_msgs;
  bool advertised;
  int selected;
  int default_select;

  void add_topic(const int id)
  {
    sub_topics.push_back(
        nh.subscribe<topic_tools::ShapeShifter>(
            "input" + std::to_string(id), 1,
            boost::bind(&message_switch::cb_topic, this, _1, id)));
  }
  void cb_select(const std_msgs::Int32::Ptr msg)
  {
    last_select_msgs = ros::Time::now();
    selected = msg->data;
  }
  void cb_topic(const boost::shared_ptr<topic_tools::ShapeShifter const> &msg, int id)
  {
    if (selected == id)
    {
      if (!advertised)
      {
        advertised = true;
        pub_topic = msg->advertise(nh, "output", 1, false);
      }
      pub_topic.publish(*msg);
    }
  }

public:
  message_switch()
    : nh("~")
  {
    sub_select = nh.subscribe("select", 1, &message_switch::cb_select, this);

    nh.param("timeout", timeout, 0.5);
    nh.param("default", default_select, 0);
    last_select_msgs = ros::Time::now();

    advertised = false;
    selected = default_select;
  }
  void spin()
  {
    int num = 1;
    add_topic(num - 1);

    ros::Rate wait(10);
    while (ros::ok())
    {
      wait.sleep();
      ros::spinOnce();
      if (ros::Time::now() - last_select_msgs > ros::Duration(timeout))
      {
        selected = default_select;
      }
      bool remain(true);
      for (auto &sub : sub_topics)
      {
        if (sub.getNumPublishers() == 0)
          remain = false;
      }
      if (remain)
      {
        num++;
        add_topic(num - 1);
      }
    }
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "message_switch");

  message_switch ms;
  ms.spin();

  return 0;
}
