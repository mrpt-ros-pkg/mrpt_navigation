/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

// ===========================================================================
//  Program: rosbag2rawlog
//  Intention: Parse bag files, save
//             as a RawLog file, easily readable by MRPT C++ programs.
//
//  Started: Hunter Laux @ SEPT-2018.
//  Maintained: JLBC @ 2018-2023
// ===========================================================================

#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/ros2bridge/imu.h>
#include <mrpt/ros2bridge/laser_scan.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/progress.h>
#include <mrpt/version.h>
#include <tf2/buffer_core.h>
#include <tf2/convert.h>
#include <tf2/exceptions.h>

#include <cv_bridge/cv_bridge.hpp>
#include <iostream>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>	// needed by tf2::fromMsg()
#include <tf2_msgs/msg/tf_message.hpp>

//#include <rosbag2_cpp/storage_options.hpp>

using namespace mrpt;
using namespace mrpt::io;
using namespace mrpt::serialization;
using namespace mrpt::system;
using namespace std;

// Declare the supported command line switches ===========
TCLAP::CmdLine cmd("rosbag2rawlog (ROS 2)", ' ', MRPT_getVersion().c_str());

TCLAP::UnlabeledValueArg<std::string> arg_input_file(
	"bags", "Input bag files (required) (*.mcap,*.db3)", true, "dataset.mcap",
	"Files", cmd);

TCLAP::ValueArg<std::string> arg_output_file(
	"o", "output", "Output dataset (*.rawlog)", true, "", "dataset_out.rawlog",
	cmd);

TCLAP::ValueArg<std::string> arg_config_file(
	"c", "config", "Config yaml file (*.yml)", true, "", "config.yml", cmd);

TCLAP::ValueArg<std::string> arg_storage_id(
	"", "storage-id", "rosbag2 storage_id format (sqlite3|mcap|...)", false,
	"mcap", "mcap", cmd);

TCLAP::ValueArg<std::string> arg_serialization_format(
	"", "serialization-format", "rosbag2 serialization format (cdr)", false,
	"cdr", "cdr", cmd);

TCLAP::SwitchArg arg_overwrite(
	"w", "overwrite", "Force overwrite target file without prompting.", cmd,
	false);

TCLAP::ValueArg<std::string> arg_base_link_frame(
	"b", "base-link",
	"Reference /tf frame for the robot frame (Default: 'base_link')", false,
	"base_link", "base_link", cmd);

using Obs = std::list<mrpt::serialization::CSerializable::Ptr>;

using CallbackFunction =
	std::function<Obs(const rosbag2_storage::SerializedBagMessage&)>;

template <typename... Args>
class RosSynchronizer
	: public std::enable_shared_from_this<RosSynchronizer<Args...>>
{
   public:
	using Tuple = std::tuple<std::shared_ptr<Args>...>;

	using Callback = std::function<Obs(const std::shared_ptr<Args>&...)>;

	RosSynchronizer(
		std::shared_ptr<tf2::BufferCore> tfBuffer, const Callback& callback)
		: m_tfBuffer(std::move(tfBuffer)), m_callback(callback)
	{
	}

	template <std::size_t... N>
	Obs signal(std::index_sequence<N...>)
	{
		auto ptr = m_callback(std::get<N>(m_cache)...);
		m_cache = {};
		return ptr;
	}

	Obs signal() { return {}; }

	template <std::size_t... N>
	bool check(std::index_sequence<N...>)
	{
		return ((std::get<N>(m_cache).get() != nullptr) && ...);
	}

	Obs checkAndSignal()
	{
		if (check(std::make_index_sequence<sizeof...(Args)>{}))
		{
			return signal();
		}
		return {};
	}

	template <size_t i>
	CallbackFunction bind()
	{
		std::shared_ptr<RosSynchronizer> ptr = this->shared_from_this();
		return [=](const rosbag2_storage::SerializedBagMessage& rosmsg) {
			if (!std::get<i>(ptr->m_cache))
			{
				/* ROS 1:
				 * 	auto pts = rosmsg.instantiate<sensor_msgs::PointCloud2>();
				 *
				 * ROS 2:
				 *  rclcpp::SerializedMessage serMsg(rosmsg->serialized_data);
				 *  auto topic = serialized_message->topic_name;
				 *  static rclcpp::Serialization<tf2_msgs::msg::TFMessage>
				 * serializer;
				 *  tf2_msgs::msg::TFMessage msg;
				 *  serializer.deserialize_message(&serMsg, &msg);
				 */

				using msg_t =
					typename std::tuple_element<i, Tuple>::type::element_type;

				// Deserialize:
				rclcpp::SerializedMessage serMsg(*rosmsg.serialized_data);
				static rclcpp::Serialization<msg_t> serializer;

				std::shared_ptr<msg_t> msg = std::make_shared<msg_t>();

				serializer.deserialize_message(&serMsg, msg.get());

				std::get<i>(ptr->m_cache) = msg;
				return ptr->checkAndSignal();
			}
			return Obs();
		};
	}

	CallbackFunction bindTfSync()
	{
		std::shared_ptr<RosSynchronizer> ptr = this->shared_from_this();
		return [=](const rosbag2_storage::SerializedBagMessage& rosmsg) {
			return ptr->checkAndSignal();
		};
	}

   private:
	std::shared_ptr<tf2::BufferCore> m_tfBuffer;
	Tuple m_cache;
	bool m_poseValid = false;
	mrpt::poses::CPose3D m_lastPose;
	Callback m_callback;
};

std::shared_ptr<tf2::BufferCore> tfBuffer;

bool findOutSensorPose(
	mrpt::poses::CPose3D& des, const std::string& target_frame,
	const std::string& source_frame,
	const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
	if (fixedSensorPose)
	{
		des = fixedSensorPose.value();
		return true;
	}

	try
	{
		ASSERT_(tfBuffer);

		geometry_msgs::msg::TransformStamped ref_to_trgFrame =
			tfBuffer->lookupTransform(
				target_frame, source_frame, {} /*latest value*/);

		tf2::Transform tf;
		tf2::fromMsg(ref_to_trgFrame.transform, tf);
		des = mrpt::ros2bridge::fromROS(tf);

		std::cout << mrpt::format(
			"[findOutSensorPose] Found pose %s -> %s: %s\n",
			source_frame.c_str(), target_frame.c_str(), des.asString().c_str());

		return true;
	}
	catch (const tf2::TransformException& ex)
	{
		std::cerr << "findOutSensorPose: " << ex.what() << std::endl;
		return false;
	}
}

Obs toPointCloud2(
	std::string_view msg, const rosbag2_storage::SerializedBagMessage& rosmsg,
	const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
	rclcpp::SerializedMessage serMsg(*rosmsg.serialized_data);
	static rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;

	sensor_msgs::msg::PointCloud2 pts;
	serializer.deserialize_message(&serMsg, &pts);

	auto ptsObs = mrpt::obs::CObservationPointCloud::Create();
	ptsObs->sensorLabel = msg;
	ptsObs->timestamp = mrpt::ros2bridge::fromROS(pts.header.stamp);

	bool sensorPoseOK = findOutSensorPose(
		ptsObs->sensorPose, pts.header.frame_id, arg_base_link_frame.getValue(),
		fixedSensorPose);
	ASSERT_(sensorPoseOK);

	// Convert points:
	std::set<std::string> fields = mrpt::ros2bridge::extractFields(pts);

	// We need X Y Z:
	if (!fields.count("x") || !fields.count("y") || !fields.count("z"))
		return {};

#if MRPT_VERSION >= 0x020b04
	if (fields.count("ring") || fields.count("time"))
	{
		// XYZIRT
		auto mrptPts = mrpt::maps::CPointsMapXYZIRT::Create();
		ptsObs->pointcloud = mrptPts;

		if (!mrpt::ros2bridge::fromROS(pts, *mrptPts))
		{
			THROW_EXCEPTION(
				"Could not convert pointcloud from ROS to CPointsMapXYZIRT");
		}
		else
		{  // converted ok:
			return {ptsObs};
		}
	}
#endif

	if (fields.count("intensity"))
	{
		// XYZI
		auto mrptPts = mrpt::maps::CPointsMapXYZI::Create();
		ptsObs->pointcloud = mrptPts;

		if (!mrpt::ros2bridge::fromROS(pts, *mrptPts))
		{
			thread_local bool warn1st = false;
			if (!warn1st)
			{
				warn1st = true;
				std::cerr << "Could not convert pointcloud from ROS to "
							 "CPointsMapXYZI. Trying with XYZ.\n";
			}
		}
		else
		{  // converted ok:
			return {ptsObs};
		}
	}

	{
		// XYZ
		auto mrptPts = mrpt::maps::CSimplePointsMap::Create();
		ptsObs->pointcloud = mrptPts;

		if (!mrpt::ros2bridge::fromROS(pts, *mrptPts))
			THROW_EXCEPTION(
				"Could not convert pointcloud from ROS to "
				"CSimplePointsMap");
	}

	return {ptsObs};
}

Obs toLidar2D(
	std::string_view msg, const rosbag2_storage::SerializedBagMessage& rosmsg,
	const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
	rclcpp::SerializedMessage serMsg(*rosmsg.serialized_data);
	static rclcpp::Serialization<sensor_msgs::msg::LaserScan> serializer;

	sensor_msgs::msg::LaserScan scan;
	serializer.deserialize_message(&serMsg, &scan);

	auto scanObs = mrpt::obs::CObservation2DRangeScan::Create();

	// Extract sensor pose from tf frames, if enabled:
	mrpt::poses::CPose3D sensorPose;
	mrpt::ros2bridge::fromROS(scan, sensorPose, *scanObs);

	scanObs->sensorLabel = msg;
	scanObs->timestamp = mrpt::ros2bridge::fromROS(scan.header.stamp);

	bool sensorPoseOK = findOutSensorPose(
		scanObs->sensorPose, scan.header.frame_id,
		arg_base_link_frame.getValue(), fixedSensorPose);
	ASSERT_(sensorPoseOK);

	return {scanObs};
}

Obs toRotatingScan(
	std::string_view msg, const rosbag2_storage::SerializedBagMessage& rosmsg,
	const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
	rclcpp::SerializedMessage serMsg(*rosmsg.serialized_data);
	static rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;

	sensor_msgs::msg::PointCloud2 pts;
	serializer.deserialize_message(&serMsg, &pts);

	// Convert points:
	std::set<std::string> fields = mrpt::ros2bridge::extractFields(pts);

	// We need X Y Z:
	if (!fields.count("x") || !fields.count("y") || !fields.count("z") ||
		!fields.count("ring"))
		return {};

	// As a structured 2D range images, if we have ring numbers:
	auto obsRotScan = mrpt::obs::CObservationRotatingScan::Create();
	const mrpt::poses::CPose3D sensorPose;

	if (!mrpt::ros2bridge::fromROS(pts, *obsRotScan, sensorPose))
	{
		THROW_EXCEPTION(
			"Could not convert pointcloud from ROS to "
			"CObservationRotatingScan. Trying another format.");
	}

	obsRotScan->sensorLabel = msg;
	obsRotScan->timestamp = mrpt::ros2bridge::fromROS(pts.header.stamp);

	bool sensorPoseOK = findOutSensorPose(
		obsRotScan->sensorPose, pts.header.frame_id,
		arg_base_link_frame.getValue(), fixedSensorPose);
	ASSERT_(sensorPoseOK);

	return {obsRotScan};
}

Obs toIMU(
	std::string_view msg, const rosbag2_storage::SerializedBagMessage& rosmsg,
	const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
	rclcpp::SerializedMessage serMsg(*rosmsg.serialized_data);
	static rclcpp::Serialization<sensor_msgs::msg::Imu> serializer;

	sensor_msgs::msg::Imu imu;
	serializer.deserialize_message(&serMsg, &imu);

	auto mrptObs = mrpt::obs::CObservationIMU::Create();

	mrptObs->sensorLabel = msg;
	mrptObs->timestamp = mrpt::ros2bridge::fromROS(imu.header.stamp);

	// Convert data:
	mrpt::ros2bridge::fromROS(imu, *mrptObs);

	bool sensorPoseOK = findOutSensorPose(
		mrptObs->sensorPose, imu.header.frame_id,
		arg_base_link_frame.getValue(), fixedSensorPose);
	ASSERT_(sensorPoseOK);

	return {mrptObs};
}

Obs toOdometry(
	std::string_view msg, const rosbag2_storage::SerializedBagMessage& rosmsg)
{
	rclcpp::SerializedMessage serMsg(*rosmsg.serialized_data);
	static rclcpp::Serialization<nav_msgs::msg::Odometry> serializer;

	nav_msgs::msg::Odometry odo;
	serializer.deserialize_message(&serMsg, &odo);

	auto mrptObs = mrpt::obs::CObservationOdometry::Create();

	mrptObs->sensorLabel = msg;
	mrptObs->timestamp = mrpt::ros2bridge::fromROS(odo.header.stamp);

	// Convert data:
	const auto pose = mrpt::ros2bridge::fromROS(odo.pose);
	mrptObs->odometry = {pose.mean.x(), pose.mean.y(), pose.mean.yaw()};

	mrptObs->hasVelocities = true;
	mrptObs->velocityLocal.vx = odo.twist.twist.linear.x;
	mrptObs->velocityLocal.vy = odo.twist.twist.linear.y;
	mrptObs->velocityLocal.omega = odo.twist.twist.angular.z;

	return {mrptObs};
}

Obs toImage(
	std::string_view msg, const rosbag2_storage::SerializedBagMessage& rosmsg,
	const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
	rclcpp::SerializedMessage serMsg(*rosmsg.serialized_data);
	static rclcpp::Serialization<sensor_msgs::msg::Image> serializer;

	auto image = std::make_shared<sensor_msgs::msg::Image>();
	serializer.deserialize_message(&serMsg, image.get());

	auto imgObs = mrpt::obs::CObservationImage::Create();

	imgObs->sensorLabel = msg;
	imgObs->timestamp = mrpt::ros2bridge::fromROS(image->header.stamp);

	auto cv_ptr = cv_bridge::toCvShare(image);

	imgObs->image = mrpt::img::CImage(cv_ptr->image, mrpt::img::DEEP_COPY);

	bool sensorPoseOK = findOutSensorPose(
		imgObs->cameraPose, image->header.frame_id,
		arg_base_link_frame.getValue(), fixedSensorPose);
	ASSERT_(sensorPoseOK);

	return {imgObs};
}

#if 0
Obs toRangeImage(
	std::string_view msg, const sensor_msgs::Image::Ptr& image,
	const sensor_msgs::CameraInfo::Ptr& cameraInfo, bool rangeIsDepth)
{
	auto cv_ptr = cv_bridge::toCvShare(image);

	// For now we are just assuming this is a range image
	if (cv_ptr->encoding == "32FC1")
	{
		auto rangeScan = mrpt::obs::CObservation3DRangeScan::Create();

		// MRPT assumes the image plane is parallel to the YZ plane, so the
		// camera is pointed in the X direction ROS assumes the image plane
		// is parallel to XY plane, so the camera is pointed in the Z
		// direction Apply a rotation to convert between these conventions.
		mrpt::math::CQuaternion<double> rot{0.5, 0.5, -0.5, 0.5};
		mrpt::poses::CPose3DQuat poseQuat(0, 0, 0, rot);
		mrpt::poses::CPose3D pose(poseQuat);
		rangeScan->setSensorPose(pose);

		rangeScan->sensorLabel = msg;
		rangeScan->timestamp = mrpt::ros2bridge::fromROS(image->header.stamp);

		rangeScan->hasRangeImage = true;
		rangeScan->rangeImage_setSize(cv_ptr->image.rows, cv_ptr->image.cols);

		rangeScan->cameraParams.nrows = cv_ptr->image.rows;
		rangeScan->cameraParams.ncols = cv_ptr->image.cols;

		std::copy(
			cameraInfo->D.begin(), cameraInfo->D.end(),
			rangeScan->cameraParams.dist.begin());

		size_t rows = cv_ptr->image.rows;
		size_t cols = cv_ptr->image.cols;
		std::copy(
			cameraInfo->K.begin(), cameraInfo->K.end(),
			rangeScan->cameraParams.intrinsicParams.begin());

		rangeScan->rangeUnits = 1e-3;
		const float inv_unit = 1.0f / rangeScan->rangeUnits;

		for (size_t i = 0; i < rows; i++)
			for (size_t j = 0; j < cols; j++)
				rangeScan->rangeImage(i, j) = static_cast<uint16_t>(
					inv_unit * cv_ptr->image.at<float>(i, j));

		rangeScan->range_is_depth = rangeIsDepth;

		return {rangeScan};
	}
	return {};
}

#endif

template <bool isStatic>
Obs toTf(
	tf2::BufferCore& tfBuffer,
	const rosbag2_storage::SerializedBagMessage& rosmsg)
{
	static rclcpp::Serialization<tf2_msgs::msg::TFMessage> tfSerializer;

	tf2_msgs::msg::TFMessage tfs;
	rclcpp::SerializedMessage msgData(*rosmsg.serialized_data);
	tfSerializer.deserialize_message(&msgData, &tfs);

	// tf2_msgs::msg::to_block_style_yaml(msg, std::cout);

	for (auto& tf : tfs.transforms)
	{
		try
		{
			tfBuffer.setTransform(tf, "bagfile", isStatic);
		}
		catch (const tf2::TransformException& ex)
		{
			std::cerr << ex.what() << std::endl;
		}
	}
	return {};
}

class Transcriber
{
   public:
	Transcriber(const mrpt::containers::yaml& config)
	{
		tfBuffer = std::make_shared<tf2::BufferCore>();

		m_lookup["/tf"].emplace_back(
			[=](const rosbag2_storage::SerializedBagMessage& rosmsg) {
				return toTf<false>(*tfBuffer, rosmsg);
			});
		m_lookup["/tf_static"].emplace_back(
			[=](const rosbag2_storage::SerializedBagMessage& rosmsg) {
				return toTf<true>(*tfBuffer, rosmsg);
			});

		for (auto& sensorNode : config["sensors"].asMap())
		{
			auto sensorName = sensorNode.first.as<std::string>();
			auto& sensor = sensorNode.second.asMap();
			const auto sensorType = sensor.at("type").as<std::string>();

			// Optional: fixed sensorPose (then ignores/don't need "tf" data):
			std::optional<mrpt::poses::CPose3D> fixedSensorPose;
			if (sensor.count("fixed_sensor_pose") != 0)
			{
				fixedSensorPose = mrpt::poses::CPose3D::FromString(
					"["s + sensor.at("fixed_sensor_pose").as<std::string>() +
					"]"s);
			}
#if 0
			if (sensorType == "CObservation3DRangeScan")
			{
				bool rangeIsDepth = sensor.count("rangeIsDepth")
										? sensor.at("rangeIsDepth").as<bool>()
										: true;
				auto callback = [=](const sensor_msgs::Image::Ptr& image,
									const sensor_msgs::CameraInfo::Ptr& info) {
					return toRangeImage(sensorName, image, info, rangeIsDepth);
				};
				using Synchronizer = RosSynchronizer<
					sensor_msgs::Image, sensor_msgs::CameraInfo>;
				auto sync = std::make_shared<Synchronizer>(
					rootFrame, tfBuffer, callback);
				m_lookup[sensor.at("depth").as<std::string>()].emplace_back(
					sync->bind<0>());
				m_lookup[sensor.at("cameraInfo").as<std::string>()]
					.emplace_back(sync->bind<1>());
				m_lookup["/tf"].emplace_back(sync->bindTfSync());
			}
#endif
			else if (sensorType == "CObservationImage")
			{
				auto callback =
					[=](const rosbag2_storage::SerializedBagMessage& m) {
						return toImage(sensorName, m, fixedSensorPose);
					};
				m_lookup[sensor.at("image_topic").as<std::string>()]
					.emplace_back(callback);
			}
			else if (sensorType == "CObservationPointCloud")
			{
				auto callback =
					[=](const rosbag2_storage::SerializedBagMessage& m) {
						return toPointCloud2(sensorName, m, fixedSensorPose);
					};
				m_lookup[sensor.at("topic").as<std::string>()].emplace_back(
					callback);
			}
			else if (sensorType == "CObservation2DRangeScan")
			{
				auto callback =
					[=](const rosbag2_storage::SerializedBagMessage& m) {
						return toLidar2D(sensorName, m, fixedSensorPose);
					};

				m_lookup[sensor.at("topic").as<std::string>()].emplace_back(
					callback);
			}
			else if (sensorType == "CObservationRotatingScan")
			{
				auto callback =
					[=](const rosbag2_storage::SerializedBagMessage& m) {
						return toRotatingScan(sensorName, m, fixedSensorPose);
					};
				m_lookup[sensor.at("topic").as<std::string>()].emplace_back(
					callback);
			}
			else if (sensorType == "CObservationIMU")
			{
				auto callback =
					[=](const rosbag2_storage::SerializedBagMessage& m) {
						return toIMU(sensorName, m, fixedSensorPose);
					};
				m_lookup[sensor.at("topic").as<std::string>()].emplace_back(
					callback);
			}
			else if (sensorType == "CObservationOdometry")
			{
				auto callback =
					[=](const rosbag2_storage::SerializedBagMessage& m) {
						return toOdometry(sensorName, m);
					};
				m_lookup[sensor.at("topic").as<std::string>()].emplace_back(
					callback);
			}
			// TODO: Handle more cases?
		}
	}

	Obs toMrpt(const rosbag2_storage::SerializedBagMessage& rosmsg)
	{
		Obs rets;
		auto topic = rosmsg.topic_name;

		if (auto search = m_lookup.find(topic); search != m_lookup.end())
		{
			for (const auto& callback : search->second)
			{
				auto obs = callback(rosmsg);
				rets.insert(rets.end(), obs.begin(), obs.end());
			}
		}
		else
		{
			if (m_unhandledTopics.count(topic) == 0)
			{
				m_unhandledTopics.insert(topic);
				std::cout << "Warning: unhandled topic '" << topic << "'"
						  << std::endl;
			}
		}
		return rets;
	};

   private:
	std::map<std::string, std::vector<CallbackFunction>> m_lookup;
	std::set<std::string> m_unhandledTopics;
};

int main(int argc, char** argv)
{
	try
	{
		printf(" rosbag2rawlog (ROS 2) - Part of the MRPT\n");
		printf(
			" MRPT C++ Library: %s - Sources timestamp: %s\n",
			MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());

		// Parse arguments:
		if (!cmd.parse(argc, argv))
			throw std::runtime_error("");  // should exit.

		auto config =
			mrpt::containers::yaml::FromFile(arg_config_file.getValue());

		auto input_bag_file = arg_input_file.getValue();
		string output_rawlog_file = arg_output_file.getValue();

		// Open input ros bag:

		rosbag2_storage::StorageOptions storage_options;

		storage_options.uri = input_bag_file;
		storage_options.storage_id = arg_storage_id.getValue();

		rosbag2_cpp::ConverterOptions converter_options;
		converter_options.input_serialization_format =
			arg_serialization_format.getValue();
		converter_options.output_serialization_format =
			arg_serialization_format.getValue();

		rosbag2_cpp::readers::SequentialReader reader;

		std::cout << "Opening: " << storage_options.uri << std::endl;
		reader.open(storage_options, converter_options);

		const std::vector<rosbag2_storage::TopicMetadata> topics =
			reader.get_all_topics_and_types();

		const auto bagMetaData = reader.get_metadata();

		const auto nEntries = bagMetaData.message_count;

		std::cout << "List of topics found in the bag (" << nEntries << " msgs"
				  << "):\n";
		for (const auto& t : topics)
			std::cout << " " << t.name << " (" << t.type << ")\n";

		// Open output:
		if (mrpt::system::fileExists(output_rawlog_file) &&
			!arg_overwrite.isSet())
		{
			cout << "Output file already exists: `" << output_rawlog_file
				 << "`, aborting. Use `-w` flag to overwrite.\n";
			return 1;
		}

		CFileGZOutputStream fil_out;
		cout << "Opening for writing: '" << output_rawlog_file << "'...\n";
		if (!fil_out.open(output_rawlog_file))
			throw std::runtime_error("Error writing file!");

		auto arch = archiveFrom(fil_out);

		size_t curEntry = 0, showProgressCnt = 0;
		Transcriber t(config);

		while (reader.has_next())
		{
			// serialized data
			auto serialized_message = reader.read_next();

			auto ptrs = t.toMrpt(*serialized_message);
			for (auto& ptr : ptrs)
			{
				arch << ptr;
			}

			curEntry++;

			if (++showProgressCnt > 100)
			{
				const double pr = (1.0 * curEntry) / nEntries;

				printf(
					"Progress: %u/%u %s %.03f%%        \r",
					static_cast<unsigned int>(curEntry),
					static_cast<unsigned int>(nEntries),
					mrpt::system::progress(pr, 50).c_str(), 100.0 * pr);
				fflush(stdout);
				showProgressCnt = 0;
			}
		}

		printf("\n");

		reader.close();

		// successful end of program.
		return 0;
	}
	catch (std::exception& e)
	{
		if (strlen(e.what()))
			std::cerr << mrpt::exception_to_str(e) << std::endl;
		return 1;
	}
}  // end of main()
