#ifndef MRPT_BRIDGE_MAP_H
#define MRPT_BRIDGE_MAP_H

#include <stdint.h>
#include <string>

namespace std
{
template <class T>
class allocator;
}

namespace std_msgs
{
template <class ContainerAllocator>
struct Header_;
typedef Header_<std::allocator<void>> Header;
}

namespace nav_msgs
{
template <class ContainerAllocator>
struct OccupancyGrid_;
typedef OccupancyGrid_<std::allocator<void>> OccupancyGrid;
}

#include <mrpt/version.h>
namespace mrpt
{
namespace maps
{
class COccupancyGridMap2D;
class CMultiMetricMap;
}
}
using mrpt::maps::COccupancyGridMap2D;
using mrpt::maps::CMultiMetricMap;

#include <mrpt/version.h>

#if MRPT_VERSION<0x199
namespace mrpt
{
namespace utils
{
class CConfigFile;
}
}
using mrpt::utils::CConfigFile;
#else
namespace mrpt
{
namespace config
{
class CConfigFile;
}
}
using mrpt::config::CConfigFile;
#endif

namespace mrpt_bridge
{
/** @name Maps, Occupancy Grid Maps: ROS <-> MRPT
 *  @{ */

/** Methods to convert between ROS msgs and MRPT objects for map datatypes.
 * @brief the map class is implemented as singeleton use map::instance
 * ()->ros2mrpt ...
  */
class MapHdl
{
   private:
	static MapHdl* instance_;  // singeleton instance
#ifdef OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
	int8_t lut_cellmrpt2ros[0xFF];  // lookup table for entry convertion
	int8_t* lut_cellmrpt2rosPtr;  // pointer to the center of the lookup table
// neede to work with neg. indexes
#else
	int8_t lut_cellmrpt2ros[0xFFFF];  // lookup table for entry convertion
	int8_t* lut_cellmrpt2rosPtr;  // pointer to the center of the lookup table
// neede to work with neg. indexes
#endif
	int8_t lut_cellros2mrpt[0xFF];  // lookup table for entry convertion
	int8_t* lut_cellros2mrptPtr;  // pointer to the center of the lookup table
	// neede to work with neg. indexes
	MapHdl();
	MapHdl(const MapHdl&);
	~MapHdl();

   public:
	/**
	  * @return returns singeleton instance
	  * @brief it creates a instance with some look up table to speed up the
	 * conversions
	  */
	static MapHdl* instance();

#ifdef OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
	const int8_t cellMrpt2Ros(int i) { return lut_cellmrpt2rosPtr[i]; }
#else
	const int16_t cellMrpt2Ros(int i) { return lut_cellmrpt2rosPtr[i]; }
#endif
	const int8_t cellRos2Mrpt(int i) { return lut_cellros2mrptPtr[i]; }
	/**
	  * loads a mprt map
	  * @return true on sucess.
	  * @param _metric_map
	  * @param _config_file
	  * @param _map_file  default: map.simplemap
	  * @param _section_name default: metricMap
	  * @param _debug default: false
	  */
	static const bool loadMap(
		CMultiMetricMap& _metric_map,
		const CConfigFile& _config_file,
		const std::string& _map_file = "map.simplemap",
		const std::string& _section_name = "metricMap", bool _debug = false);
};

/**
  * converts ros msg to mrpt object
  * @return true on sucessful conversion, false on any error.
  * @param src
  * @param des
  */
bool convert(const nav_msgs::OccupancyGrid& src, COccupancyGridMap2D& des);

/**
  * converts mrpt object to ros msg and updates the msg header
  * @return true on sucessful conversion, false on any error.
  * @param src
  * @param des
  * @param header
  */
bool convert(
	const COccupancyGridMap2D& src, nav_msgs::OccupancyGrid& msg,
	const std_msgs::Header& header);
/**
  * converts mrpt object to ros msg
  * @return true on sucessful conversion, false on any error.
  * @param src
  * @param des
  * @param header
  */
bool convert(const COccupancyGridMap2D& src, nav_msgs::OccupancyGrid& msg);

/** @} */

}  // namespace mrpt_bridge

#endif  // MRPT_BRIDGE_MAP_H
