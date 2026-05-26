#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "c3_sonar_driver/msg/sonar_detect.hpp"

namespace c3_sonar_driver
{

	class DaveSonarAdapterNode : public rclcpp::Node
	{
	public:
		DaveSonarAdapterNode()
			: Node("dave_sonar_adapter_node")
		{
			input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", "/multibeam_sonar_point_cloud");
			output_detect_topic_ = declare_parameter<std::string>("output_detect_topic", "/sonar/detect");
			min_points_for_valid_detect_ = declare_parameter<int>("min_points_for_valid_detect", 20);
			default_confidence_ = declare_parameter<double>("default_confidence", 0.65);

			cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
				input_cloud_topic_, rclcpp::SensorDataQoS(),
				[this](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
				{ onCloud(msg); });

			detect_pub_ = create_publisher<msg::SonarDetect>(output_detect_topic_, 10);

			RCLCPP_INFO(get_logger(), "dave_sonar_adapter_node started. input=%s output=%s",
						input_cloud_topic_.c_str(), output_detect_topic_.c_str());
		}

	private:
		void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
		{
			if (!cloud)
			{
				return;
			}

			if (cloud->width * cloud->height == 0U || cloud->data.empty())
			{
				return;
			}

			// Simple centroid-based detection from DAVE point cloud.
			double sum_x = 0.0;
			double sum_y = 0.0;
			double sum_z = 0.0;
			std::size_t valid_count = 0;

			sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
			sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
			sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");

			for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
			{
				const float x = *iter_x;
				const float y = *iter_y;
				const float z = *iter_z;

				if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
				{
					continue;
				}

				const double r2 = static_cast<double>(x) * x + static_cast<double>(y) * y + static_cast<double>(z) * z;
				if (r2 < 1e-8)
				{
					continue;
				}

				sum_x += x;
				sum_y += y;
				sum_z += z;
				++valid_count;
			}

			if (static_cast<int>(valid_count) < min_points_for_valid_detect_)
			{
				return;
			}

			const double cx = sum_x / static_cast<double>(valid_count);
			const double cy = sum_y / static_cast<double>(valid_count);
			const double cz = sum_z / static_cast<double>(valid_count);

			msg::SonarDetect detect;
			detect.header = cloud->header;
			detect.detect_id = ++last_detect_id_;
			detect.position.x = cx;
			detect.position.y = cy;
			detect.position.z = cz;

			detect.velocity.x = 0.0;
			detect.velocity.y = 0.0;
			detect.velocity.z = 0.0;

			detect.range_m = static_cast<float>(std::sqrt(cx * cx + cy * cy + cz * cz));
			detect.bearing_rad = static_cast<float>(std::atan2(cy, cx));

			const double confidence = std::clamp(default_confidence_, 0.0, 1.0);
			detect.confidence = static_cast<float>(confidence);
			detect.cloud = *cloud;

			detect_pub_->publish(detect);
		}

		std::string input_cloud_topic_;
		std::string output_detect_topic_;
		int min_points_for_valid_detect_{20};
		double default_confidence_{0.65};

		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
		rclcpp::Publisher<msg::SonarDetect>::SharedPtr detect_pub_;

		uint32_t last_detect_id_{0};
	};

} // namespace c3_sonar_driver

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<c3_sonar_driver::DaveSonarAdapterNode>());
	rclcpp::shutdown();
	return 0;
}
