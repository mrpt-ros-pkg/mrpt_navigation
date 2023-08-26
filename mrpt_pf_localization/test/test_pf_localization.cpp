/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2023, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt_pf_localization/mrpt_pf_localization_core.h>

#include <thread>

//#define RUN_TESTS_WITH_GUI

const char* TEST_PARAMS_YAML_FILE =
	MRPT_LOCALIZATION_SOURCE_DIR "/params/default.config.yaml";

const char* TEST_MAP_CONFIG_FILE =
	MRPT_LOCALIZATION_SOURCE_DIR "/params/map-occgrid2d.ini";

const char* TEST_SIMPLEMAP_FILE =
	MRPT_LOCALIZATION_SOURCE_DIR "/tutorial/map.simplemap";

const char* TEST_RAWLOG_FILE =
	MRPT_LOCALIZATION_SOURCE_DIR "/tutorial/driving_in_office_obs.rawlog";

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
	PFLocalizationCore loc;

	auto p = mrpt::containers::yaml::FromFile(TEST_PARAMS_YAML_FILE);
	mrpt::containers::yaml params =
		p["mrpt_pf_localization_node"]["ros__parameters"];

#if !defined(RUN_TESTS_WITH_GUI)
	// For running tests, disable GUI (comment out to see the GUI live):
	params["gui_enable"] = false;
#endif

	// Load params:
	loc.init_from_yaml(params);

	// Check params:
	EXPECT_EQ(loc.getParams().initial_particle_count, 2500U);

	// Check that we are still in UNINITILIZED state, even after stepping,
	// since we don't have a map yet:
	EXPECT_EQ(loc.getState(), PFLocalizationCore::State::UNINITIALIZED);
	loc.step();
	EXPECT_EQ(loc.getState(), PFLocalizationCore::State::UNINITIALIZED);

	// Now, load a map:
	bool loadOk =
		loc.set_map_from_simple_map(TEST_MAP_CONFIG_FILE, TEST_SIMPLEMAP_FILE);
	EXPECT_TRUE(loadOk);

	// Now, we should transition to TO_INITIALIZE:
	loc.step();
	EXPECT_EQ(loc.getState(), PFLocalizationCore::State::TO_BE_INITIALIZED);

	// And next iter, we should be running with all particles around:
	loc.step();
	EXPECT_EQ(loc.getState(), PFLocalizationCore::State::RUNNING);

	// TODO: Check that number of particles is high:

	// Run for a small dataset:
	mrpt::obs::CRawlog dataset;
	dataset.loadFromRawLogFile(TEST_RAWLOG_FILE);
	EXPECT_GT(dataset.size(), 20U);

	double lastStepTime = 0.0;
	for (const auto& observation : dataset)
	{
		const auto obs =
			std::dynamic_pointer_cast<mrpt::obs::CObservation>(observation);

		loc.on_observation(obs);

		const double thisObsTim = mrpt::Clock::toDouble(obs->timestamp);

		if (thisObsTim - lastStepTime > 0.10)
		{
			lastStepTime = thisObsTim;
			loc.step();

			if (loc.getParams().gui_enable)
				std::this_thread::sleep_for(std::chrono::milliseconds(50));
		}
	}
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
