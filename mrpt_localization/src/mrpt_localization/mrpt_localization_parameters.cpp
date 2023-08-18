/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2023, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <mrpt_localization/mrpt_localization.h>
#include <mrpt_localization/mrpt_localization_defaults.h>

PFLocalization::Parameters::Parameters(PFLocalization* p)
	: debug(MRPT_LOCALIZATION_DEFAULT_DEBUG),
	  gui_mrpt(MRPT_LOCALIZATION_DEFAULT_GUI_MRPT),
	  ini_file(MRPT_LOCALIZATION_DEFAULT_INI_FILE),
	  map_file(MRPT_LOCALIZATION_DEFAULT_MAP_FILE),
	  sensor_sources(MRPT_LOCALIZATION_DEFAULT_SENSOR_SOURCES),
	  use_motion_model_default_options(&p->use_motion_model_default_options_),
	  motion_model_options(&p->motion_model_options_),
	  motion_model_default_options(&p->motion_model_default_options_)
{
}
