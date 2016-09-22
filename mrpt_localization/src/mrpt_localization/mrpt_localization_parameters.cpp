/***********************************************************************************
 * Revised BSD License                                                             *
 * Copyright (c) 2014, Markus Bader <markus.bader@tuwien.ac.at>                    *
 * All rights reserved.                                                            *
 *                                                                                 *
 * Redistribution and use in source and binary forms, with or without              *
 * modification, are permitted provided that the following conditions are met:     *
 *     * Redistributions of source code must retain the above copyright            *
 *       notice, this list of conditions and the following disclaimer.             *
 *     * Redistributions in binary form must reproduce the above copyright         *
 *       notice, this list of conditions and the following disclaimer in the       *
 *       documentation and/or other materials provided with the distribution.      *
 *     * Neither the name of the Vienna University of Technology nor the           *
 *       names of its contributors may be used to endorse or promote products      *
 *       derived from this software without specific prior written permission.     *
 *                                                                                 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND *
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED   *
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE          *
 * DISCLAIMED. IN NO EVENT SHALL Markus Bader BE LIABLE FOR ANY                    *
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES      *
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;    *
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND     *
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT      *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS   *
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                    *                       *
 ***********************************************************************************/

#include <mrpt_localization/mrpt_localization.h>
#include <mrpt_localization/mrpt_localization_defaults.h>

PFLocalization::Parameters::Parameters(PFLocalization *p) :
    debug(MRPT_LOCALIZATION_DEFAULT_DEBUG),
    gui_mrpt(MRPT_LOCALIZATION_DEFAULT_GUI_MRPT),
    iniFile(MRPT_LOCALIZATION_DEFAULT_INI_FILE),
    mapFile(MRPT_LOCALIZATION_DEFAULT_MAP_FILE),
    sensorSources(MRPT_LOCALIZATION_DEFAULT_SENSOR_SOURCES),
    use_motion_model_default_options(&p->use_motion_model_default_options_),
    motion_model_options(&p->motion_model_options_),
    motion_model_default_options(&p->motion_model_default_options_)
{
}
