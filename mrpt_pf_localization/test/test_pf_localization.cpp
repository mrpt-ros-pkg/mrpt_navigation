/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/Generator.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt_pf_localization/mrpt_pf_localization_core.h>

#include <thread>

struct TestParams
{
	const bool RUN_TESTS_WITH_GUI = mrpt::get_env("RUN_TESTS_WITH_GUI", false);

	// x y z yaw_deg pitch_deg roll_deg
	const std::string TEST_FINAL_GT_POSE = mrpt::get_env<std::string>(
		"TEST_FINAL_GT_POSE", "[-9.03 4.5 0 4.3 0 0]");

	const double TEST_CONVERGENCE_TOLERANCE =
		mrpt::get_env<double>("TEST_CONVERGENCE_TOLERANCE", 0.25);

	const std::string TEST_MM_FILE =
		mrpt::get_env<std::string>("TEST_MM_FILE", "");

	const char* DEFAULT_TEST_PF_YAML_FILE =
		MRPT_LOCALIZATION_SOURCE_DIR "/params/default.config.yaml";

	const std::string TEST_PF_YAML_FILE = mrpt::get_env<std::string>(
		"TEST_PF_YAML_FILE", DEFAULT_TEST_PF_YAML_FILE);

	const std::string TEST_RELOCALIZATION_YAML_FILE =
		mrpt::get_env<std::string>(
			"TEST_RELOCALIZATION_YAML_FILE", MRPT_LOCALIZATION_SOURCE_DIR
			"/params/default-relocalization-pipeline.yaml");

	const std::string TEST_INPUT_OBSERVATION_PIPELINE_YAML_FILE =
		mrpt::get_env<std::string>("TEST_INPUT_OBSERVATION_PIPELINE_YAML_FILE");

	const std::string TEST_INPUT_OBSERVATION_LAYER =
		mrpt::get_env<std::string>("TEST_INPUT_OBSERVATION_LAYER");

	const char* TEST_MAP_CONFIG_FILE =
		MRPT_LOCALIZATION_SOURCE_DIR "/params/map-occgrid2d.ini";

	const char* TEST_SIMPLEMAP_FILE = MRPT_LOCALIZATION_SOURCE_DIR
		"/../mrpt_tutorials/maps/gh25_simulated.simplemap";

	const std::string TEST_RAWLOG_FILE = mrpt::get_env<std::string>(
		"TEST_RAWLOG_FILE", MRPT_LOCALIZATION_SOURCE_DIR
		"/../mrpt_tutorials/datasets/driving_in_office_obs.rawlog");

	const size_t TEST_SKIP_FIRST_N =
		mrpt::get_env<size_t>("TEST_SKIP_FIRST_N", 0);
};

TEST(PF_Localization, InitState)
{
	PFLocalizationCore loc;

	for (int i = 0; i < 10; i++)
	{
		EXPECT_EQ(loc.getState(), PFLocalizationCore::State::UNINITIALIZED);
		loc.step();
	}
}

TEST(PF_Localization, RunRealDataset)
{
	TestParams _;

	PFLocalizationCore loc;

	if (mrpt::get_env<bool>("VERBOSE"))
		loc.setMinLoggingLevel(mrpt::system::LVL_DEBUG);

	auto p = mrpt::containers::yaml::FromFile(_.TEST_PF_YAML_FILE);
	mrpt::containers::yaml params = p["/**"]["ros__parameters"];

	auto relocParams =
		mrpt::containers::yaml::FromFile(_.TEST_RELOCALIZATION_YAML_FILE);

	// For running tests, disable GUI (comment out to see the GUI live):
	params["gui_enable"] = _.RUN_TESTS_WITH_GUI;

	// Load params:
	loc.init_from_yaml(params, relocParams);

	// Mimic the pipeline processing pipeline in this test app, where
	// the input observations are raw scans:
	mp2p_icp_filters::FilterPipeline perObsPipeline, obsFinalPipeline;
	mp2p_icp_filters::GeneratorSet obsGenerators;

	if (!_.TEST_INPUT_OBSERVATION_PIPELINE_YAML_FILE.empty())
	{
		const auto y = mrpt::containers::yaml::FromFile(
			_.TEST_INPUT_OBSERVATION_PIPELINE_YAML_FILE);

		obsGenerators = mp2p_icp_filters::generators_from_yaml(y["generators"]);

		perObsPipeline =
			mp2p_icp_filters::filter_pipeline_from_yaml(y["per_observation"]);

		obsFinalPipeline =
			mp2p_icp_filters::filter_pipeline_from_yaml(y["final"]);
	}

	// Check params:
	const bool custom_yaml_file =
		_.DEFAULT_TEST_PF_YAML_FILE != _.TEST_PF_YAML_FILE;

	if (!custom_yaml_file)
	{
		EXPECT_EQ(loc.getParams().initial_particles_per_m2, 50U);
	}

	// Check that we are still in UNINITILIZED state, even after stepping,
	// since we don't have a map yet:
	if (!custom_yaml_file)
	{
		EXPECT_EQ(loc.getState(), PFLocalizationCore::State::UNINITIALIZED);
	}

	loc.step();

	if (!custom_yaml_file)
	{
		EXPECT_EQ(loc.getState(), PFLocalizationCore::State::UNINITIALIZED);
	}

	// Now, load a map:
	if (_.TEST_MM_FILE.empty())
	{
		bool loadOk = loc.set_map_from_simple_map(
			_.TEST_MAP_CONFIG_FILE, _.TEST_SIMPLEMAP_FILE);
		EXPECT_TRUE(loadOk);
	}
	else
	{
		mp2p_icp::metric_map_t mm;
		bool loadOk = mm.load_from_file(_.TEST_MM_FILE);
		EXPECT_TRUE(loadOk);

		loc.set_map_from_metric_map(mm);
	}

	// Now, we should transition to TO_INITIALIZE:
	loc.step();
	if (!custom_yaml_file)
	{
		EXPECT_EQ(loc.getState(), PFLocalizationCore::State::TO_BE_INITIALIZED);
	}

	// And next iter, we should be running with all particles around:
	loc.step();
	if (!custom_yaml_file)
	{
		EXPECT_EQ(loc.getState(), PFLocalizationCore::State::RUNNING);
	}

	// Check that number of particles is high:
#if 0  // This won't work with the mola_relocalization method
	ASSERT_GT(loc.getLastPoseEstimation()->size(), 500U);
#endif

	// Run for a small dataset:
	mrpt::obs::CRawlog dataset;
	dataset.loadFromRawLogFile(_.TEST_RAWLOG_FILE);
	EXPECT_GT(dataset.size(), 20U);

	double lastStepTime = 0.0;
	size_t datasetIndex = 0;
	for (const auto& observation : dataset)
	{
		datasetIndex++;

		if (datasetIndex <= _.TEST_SKIP_FIRST_N) continue;

		const auto obs =
			std::dynamic_pointer_cast<mrpt::obs::CObservation>(observation);
		if (!obs) continue;

		// processing for raw input data?
		mrpt::obs::CObservation::Ptr obsToProcess;

		const bool obsIsPointCloud =
			IS_CLASS(*obs, mrpt::obs::CObservationPointCloud) ||
			IS_CLASS(*obs, mrpt::obs::CObservation3DRangeScan);

		if (!perObsPipeline.empty() && obsIsPointCloud)
		{
			auto newPc = mrpt::obs::CObservationPointCloud::Create();
			newPc->timestamp = obs->timestamp;
			newPc->sensorLabel = obs->sensorLabel;

			mp2p_icp::metric_map_t obsMap =
				mp2p_icp_filters::apply_generators(obsGenerators, *obs);

			mp2p_icp_filters::apply_filter_pipeline(perObsPipeline, obsMap);
			mp2p_icp_filters::apply_filter_pipeline(obsFinalPipeline, obsMap);

			newPc->pointcloud =
				obsMap.point_layer(_.TEST_INPUT_OBSERVATION_LAYER);

			obsToProcess = newPc;
		}
		else
		{
			obsToProcess = obs;
		}

		// run PF:
		loc.on_observation(obsToProcess);

		const double thisObsTim = mrpt::Clock::toDouble(obs->timestamp);

		if (thisObsTim - lastStepTime > 0.10)
		{
			lastStepTime = thisObsTim;
			loc.step();

			if (loc.getParams().gui_enable)
				std::this_thread::sleep_for(std::chrono::milliseconds(50));
		}
	}

	if (auto pe = loc.getLastPoseEstimation(); pe)
	{
		// Check PF convergence to ground truth
		const auto [cov, mean] = pe->getCovarianceAndMean();

		const double std_x = std::sqrt(cov(0, 0));
		const double std_y = std::sqrt(cov(1, 1));
		const double std_yaw = std::sqrt(cov(3, 3));

		EXPECT_LT(std_x, 0.5);
		EXPECT_LT(std_y, 0.5);
		EXPECT_LT(std_yaw, 0.1);

		using namespace mrpt::literals;	 // _deg

		const auto gtPose =
			mrpt::poses::CPose3D::FromString(_.TEST_FINAL_GT_POSE);

		EXPECT_LT(
			(mean - gtPose).asVectorVal().norm(), _.TEST_CONVERGENCE_TOLERANCE)
			<< "mean: " << mean << "\n"
			<< "gtPose: " << gtPose << "\n";
	}

	if (_.RUN_TESTS_WITH_GUI)
	{
		std::this_thread::sleep_for(std::chrono::seconds(5));
	}
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
