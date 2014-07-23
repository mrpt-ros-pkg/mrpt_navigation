#ifndef MRPT_BRIDGE_MAP_H
#define MRPT_BRIDGE_MAP_H

#include <stdint.h>
#include <string>

namespace std{
    template <class T> class allocator;
}

namespace std_msgs{
    template <class ContainerAllocator> struct Header_;
    typedef Header_<std::allocator<void> > Header;
}

namespace nav_msgs{
    template <class ContainerAllocator> struct OccupancyGrid_;
    typedef OccupancyGrid_<std::allocator<void> > OccupancyGrid;
}


namespace mrpt
{
namespace slam
{
class COccupancyGridMap2D;
class CMultiMetricMap;
}
namespace utils {
class CConfigFile;
}
}

namespace mrpt_bridge
{

/** Methods to convert between ROS msgs and MRPT objects for map datatypes.
 * @brief the map class is implemented as singeleton use map::instance ()->ros2mrpt ...
  */
class map
{
private:
    static map* instance_; // singeleton instance
#ifdef  OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
    int8_t lut_mrpt2ros[0xFF]; // lookup table for entry convertion
    int8_t *lut_mrpt2rosPtr;   // pointer to the center of the lookup table neede to work with neg. indexes
#else
    int16_t lut_mrpt2ros[0xFFFF]; // lookup table for entry convertion
    int16_t *lut_mrpt2rosPtr; // pointer to the center of the lookup table neede to work with neg. indexes
#endif
    map ( );
    map ( const map& );
    ~map ();
public:
    /**
      * @return returns singeleton instance
      * @brief it creates a instance with some look up table to speed up the conversions
      */
    static map* instance ();
    
    /**
      * converts ros msg to mrpt object
      * @return true on sucessful conversion, false on any error.
      * @param src
      * @param des
      */
    bool ros2mrpt ( const nav_msgs::OccupancyGrid  &src, mrpt::slam::COccupancyGridMap2D  &des );

    /**
      * converts mrpt object to ros msg and updates the msg header
      * @return true on sucessful conversion, false on any error.
      * @param src
      * @param des
      * @param header
      */
    bool mrpt2ros (
        const mrpt::slam::COccupancyGridMap2D &src,
        nav_msgs::OccupancyGrid &msg,
        const std_msgs::Header &header);
    /**
      * converts mrpt object to ros msg
      * @return true on sucessful conversion, false on any error.
      * @param src
      * @param des
      * @param header
      */
    bool mrpt2ros (
        const mrpt::slam::COccupancyGridMap2D &src,
        nav_msgs::OccupancyGrid &msg);


    /**
      * loads a mprt map
      * @return true on sucess.
      * @param _metric_map
      * @param _config_file
      * @param _map_file  default: map.simplemap
      * @param _section_name default: metricMap
      * @param _debug default: false
      */
    static const bool loadMap(mrpt::slam::CMultiMetricMap &_metric_map, const mrpt::utils::CConfigFile &_config_file, const std::string &_map_file="map.simplemap", const std::string &_section_name="metricMap", bool _debug = false);
};


}; //namespace mrpt_bridge

#endif //MRPT_BRIDGE_MAP_H
