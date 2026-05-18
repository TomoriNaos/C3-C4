#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace c3_drone_driver
{

	class Config
	{
	private:
		inline static std::shared_ptr<Config> config_ptr_{nullptr};
		cv::FileStorage file_;

		Config() = default;

		static std::shared_ptr<Config> Create()
		{
			return std::shared_ptr<Config>(new Config());
		}

	public:
		~Config() = default;

		static bool SetParameterFile(const std::string &filename)
		{
			auto cfg = Create();
			cfg->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
			if (!cfg->file_.isOpened())
			{
				return false;
			}
			config_ptr_ = cfg;
			return true;
		}

		template <typename T>
		static T Get(const std::string &key)
		{
			return T(config_ptr_->file_[key]);
		}

		template <typename T>
		static T GetOr(const std::string &key, const T &fallback)
		{
			if (!config_ptr_)
			{
				return fallback;
			}
			const cv::FileNode node = config_ptr_->file_[key];
			if (node.empty())
			{
				return fallback;
			}
			return T(node);
		}
	};

	template <>
	inline std::vector<double> Config::Get<std::vector<double>>(const std::string &key)
	{
		if (!config_ptr_)
		{
			return {};
		}
		const cv::FileNode node = config_ptr_->file_[key];
		if (node.type() != cv::FileNode::SEQ)
		{
			return {};
		}
		std::vector<double> vec;
		for (cv::FileNodeIterator it = node.begin(); it != node.end(); ++it)
		{
			vec.push_back(static_cast<double>(*it));
		}
		return vec;
	}

} // namespace c3_drone_driver
