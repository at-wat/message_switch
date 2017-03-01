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

	void add_topic(const int id)
	{
		sub_topics.push_back(
				nh.subscribe<topic_tools::ShapeShifter>("input" + std::to_string(id), 1, 
				boost::bind(&message_switch::cb_topic, this, _1, id)));
	}
	void cb_select(const std_msgs::Int32::Ptr msg)
	{
		last_select_msgs = ros::Time::now();
		selected = msg->data;
	};
	void cb_topic(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg, int id)
	{
		if(selected == id)
		{
			if(!advertised)
			{
				advertised = true;
				pub_topic = msg->advertise(nh, "output", 1, false);
			}
			pub_topic.publish(*msg);
		}
	};

public:
	message_switch():
		nh("~")
	{
		sub_select = nh.subscribe("select", 1, &message_switch::cb_select, this);

		nh.param("timeout", timeout, 0.5);
		last_select_msgs = ros::Time::now();

		advertised = false;
		selected = 0;
	}
	void spin()
	{
		int num = 1;
		add_topic(num - 1);

		ros::Rate wait(10);
		while(ros::ok())
		{
			wait.sleep();
			ros::spinOnce();
			if(ros::Time::now() - last_select_msgs > ros::Duration(timeout))
			{
				selected = 0;
			}
			bool remain(true);
			for(auto &sub: sub_topics)
			{
				if(sub.getNumPublishers() == 0)
					remain = false;
			}
			if(remain)
			{
				num ++;
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


