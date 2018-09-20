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

class MessageSwitch
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::vector<ros::Subscriber> sub_topics_;
  ros::Subscriber sub_select_;
  ros::Publisher pub_topic_;
  ros::Timer timer_;
  double timeout_;
  ros::Time last_select_msgs_;
  bool advertised_;
  int selected_;
  int default_select_;

  void add_topic(const int id)
  {
    sub_topics_.push_back(
        nh_.subscribe<topic_tools::ShapeShifter>(
            "input" + std::to_string(id), 1,
            boost::bind(&MessageSwitch::cb_topic, this, _1, id)));
  }
  void cb_select(const std_msgs::Int32::Ptr msg)
  {
    last_select_msgs_ = ros::Time::now();
    selected_ = msg->data;
  }
  void cb_topic(const boost::shared_ptr<topic_tools::ShapeShifter const> &msg, int id)
  {
    if (selected_ == id)
    {
      if (!advertised_)
      {
        advertised_ = true;
        pub_topic_ = msg->advertise(nh_, "output", 1, false);
      }
      pub_topic_.publish(*msg);
    }
  }

public:
  MessageSwitch()
    : nh_()
    , pnh_("~")
  {
    sub_select_ = nh_.subscribe("select", 1, &MessageSwitch::cb_select, this);

    pnh_.param("timeout", timeout_, 0.5);
    pnh_.param("default", default_select_, 0);
    last_select_msgs_ = ros::Time::now();

    advertised_ = false;
    selected_ = default_select_;

    timer_ = nh.createTimer(ros::Duration(0.1), &MessageSwitch::cbTimer, this);
    add_topic();
  }
  void cbTimer(const ros::TimerEvent &event)
  {
    if (ros::Time::now() - last_select_msgs_ > ros::Duration(timeout_))
    {
      selected_ = default_select_;
    }
    bool remain(true);
    for (auto &sub : sub_topics_)
    {
      if (sub.getNumPublishers() == 0)
        remain = false;
    }
    if (remain)
    {
      add_topic();
    }
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "message_switch");

  MessageSwitch ms;
  ros::spin();

  return 0;
}
