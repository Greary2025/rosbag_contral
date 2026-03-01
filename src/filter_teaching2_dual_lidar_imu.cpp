#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <cstdio>
#include <map>
#include <set>
#include <string>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "filter_teaching2_dual_lidar_imu");

	const std::string input_bag = "/mnt/c/Users/Greary/Documents/rosbag/real2_bag/teaching2_dual.bag";
	const std::string output_bag = "/mnt/c/Users/Greary/Documents/rosbag/real2_bag/teaching2_dual_lidar_imu.bag";

	// Keep only LiDAR/IMU-related topics and static TF
	const std::set<std::string> keep_topics = {
		"/xianfeng/lidar",
		"/xianfeng/imu",
		"/gensui/lidar",
		"/gensui/imu",
		"/tf_static"
	};

	// Force IMU frame_id to imu_link to avoid double-transforming
	const std::map<std::string, std::string> imu_frame_map = {
		{"/xianfeng/imu", "xianfeng/imu_link"},
		{"/gensui/imu", "gensui/imu_link"}
	};

	rosbag::Bag in_bag;
	rosbag::Bag out_bag;

	try {
		in_bag.open(input_bag, rosbag::bagmode::Read);
	} catch (const rosbag::BagException& ex) {
		ROS_ERROR("Failed to open input bag: %s", ex.what());
		return 1;
	}

	// Ensure clean output
	std::remove(output_bag.c_str());
	try {
		out_bag.open(output_bag, rosbag::bagmode::Write);
	} catch (const rosbag::BagException& ex) {
		ROS_ERROR("Failed to open output bag: %s", ex.what());
		return 1;
	}

	rosbag::View view(in_bag);

	std::size_t processed = 0;
	std::size_t written = 0;

	BOOST_FOREACH (rosbag::MessageInstance const m, view)
	{
		++processed;
		const std::string topic = m.getTopic();
		if (keep_topics.find(topic) == keep_topics.end()) {
			if (processed % 50000 == 0) {
				ROS_INFO("Processed %zu messages, wrote %zu", processed, written);
			}
			continue;
		}

		// IMU: rewrite frame_id
		if (topic == "/xianfeng/imu" || topic == "/gensui/imu") {
			auto imu_it = imu_frame_map.find(topic);
			sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
			if (imu_msg && imu_it != imu_frame_map.end()) {
				sensor_msgs::Imu imu_copy = *imu_msg;
				imu_copy.header.frame_id = imu_it->second;
				out_bag.write(topic, m.getTime(), imu_copy);
				++written;
			}
		// LiDAR point clouds
		} else if (topic == "/xianfeng/lidar" || topic == "/gensui/lidar") {
			sensor_msgs::PointCloud2::ConstPtr cloud = m.instantiate<sensor_msgs::PointCloud2>();
			if (cloud) {
				out_bag.write(topic, m.getTime(), *cloud);
				++written;
			}
		// Static TF (write both vehicles)
		} else if (topic == "/tf_static") {
			tf2_msgs::TFMessage::ConstPtr tfm = m.instantiate<tf2_msgs::TFMessage>();
			if (tfm) {
				out_bag.write(topic, m.getTime(), *tfm);
				++written;
				// Duplicate tf_static for the other vehicle if missing
				tf2_msgs::TFMessage tfm_copy = *tfm;
				for (auto& tr : tfm_copy.transforms) {
					std::string parent = tr.header.frame_id;
					std::string child = tr.child_frame_id;
					const bool parent_slash = !parent.empty() && parent[0] == '/';
					const bool child_slash = !child.empty() && child[0] == '/';
					if (parent_slash) parent = parent.substr(1);
					if (child_slash) child = child.substr(1);
					if (parent.rfind("xianfeng/", 0) == 0) parent.replace(0, 9, "gensui/");
					if (child.rfind("xianfeng/", 0) == 0) child.replace(0, 9, "gensui/");
					tr.header.frame_id = parent_slash ? "/" + parent : parent;
					tr.child_frame_id = child_slash ? "/" + child : child;
				}
				out_bag.write(topic, m.getTime(), tfm_copy);
				++written;
			}
		}

		if (processed % 50000 == 0) {
			ROS_INFO("Processed %zu messages, wrote %zu", processed, written);
		}
	}

	in_bag.close();
	out_bag.close();

	ROS_INFO("Done. Total processed: %zu, written: %zu", processed, written);
	ROS_INFO("Output: %s", output_bag.c_str());
	return 0;
}
