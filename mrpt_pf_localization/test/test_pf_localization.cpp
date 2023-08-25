/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2023, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt_pf_localization/mrpt_pf_localization_core.h>

TEST(PF_Localization, InitState)
{
	PFLocalizationCore loc;

	for (int i = 0; i < 10; i++)
	{
		EXPECT_EQ(loc.getState(), PFLocalizationCore::State::UNINITIALIZED);
		loc.step();
	}
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
