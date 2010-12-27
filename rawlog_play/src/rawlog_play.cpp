/* +---------------------------------------------------------------------------+
   | rawlog_play                                                               |
   |   Copyright (C) 2010-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   | rawlog_play is free software: you can redistribute it and/or modify       |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   rawlog_play is distributed in the hope that it will be useful,          |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with rawlog_play.  If not, see <http://www.gnu.org/licenses/>.  |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

/* Author: Jose Luis Blanco     */

/**

@mainpage rawlog_play

@htmlinclude manifest.html

@b rawlog_play takes datasets stored in the MRPT rawlog format and publishes it as ROS messages.
    It supports camera and laser sensors for now.

<hr>

@section topic ROS topics

Subscribes to (name/type):
- (none)

Publishes to (name/type):
- @b "<SENSOR_LABEL>"/<a href="../../sensor_msgs/html/classstd__msgs_1_1LaserScan.html">sensor_msgs/LaserScan</a> : laser scans,
where <SENSOR_LABEL1> corresponds to the name actually stored in the dataset. If the dataset contains scans from several scanners,
this package will publish several different topics, one for each laser.

@section services
-


@section parameters ROS parameters

This package reads the following parameters from the parameter server:

- @b "~rawlog_file": @b [string, optional] The file to read. It can be any MRPT rawlog file, either raw or GZ-compressed.
If the parameter is not present this module will no nothing until a service is invoked.

*/


#include <ros/ros.h>
#include <list>
#include <sstream>

#include <std_msgs/String.h>

// My own services:
#include <rawlog_play/SetRawlogFile.h>
#include <rawlog_play/Play.h>
#include <rawlog_play/Pause.h>
#include <rawlog_play/Resume.h>

// mrpt<->ros converter classes
#include <mrpt_bridge/laser_scan.h>

#include <mrpt/utils/CFileGZInputStream.h>

using namespace std;
using namespace mrpt::utils;


/*************************************************
			Global vars
*************************************************/
string              rawlog_fil;
CFileGZInputStream  rawlog_fil_stream;

/*************************************************
 Service: SetRawlogFile
*************************************************/
bool srv_setrawlogfile(rawlog_play::SetRawlogFile::Request  &req,
         rawlog_play::SetRawlogFile::Response &res )
{
//  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}


/*************************************************
                   main
*************************************************/
int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "rawlog_play");

	// Read parameters
	ros::NodeHandle private_nh_("~");
	if (!private_nh_.getParam(string("rawlog_file"),rawlog_fil))
	{
		// None provided.
	}

	// NodeHandle is the main access point to communications with the ROS system.
	ros::NodeHandle node;

	// Create services:
	ros::ServiceServer srv1 = node.advertiseService("set_rawlog_file", srv_setrawlogfile);


	// The list of published topics:
	std::list<ros::Publisher> pub_topics;
	pub_topics.push_back( node.advertise<std_msgs::String>("chatter", 1000) );

	ros::Rate loop_rate(1000);  // max. loop rate in Hz

	while (ros::ok())
	{
		std_msgs::String msg;

		std::stringstream ss;
		ss << "hello world ";
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());

		pub_topics.begin()->publish(msg);

		// End of loop.
		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0; // all ok.
}
