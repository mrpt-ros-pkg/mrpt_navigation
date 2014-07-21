#ifndef MRPT_BRIDGE_MAP_H
#define MRPT_BRIDGE_MAP_H

#include <nav_msgs/OccupancyGrid.h>

namespace mrpt
{
namespace slam
{
class COccupancyGridMap2D;
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
      * \return returns singeleton instance
      * \brief it creates a instance with some look up table to speed up the conversions
      */
    static map* instance ();
    
    /**
      * \return true on sucessful conversion, false on any error.
      * \sa mrpt2ros
      */
    bool ros2mrpt ( const nav_msgs::OccupancyGrid  &des, mrpt::slam::COccupancyGridMap2D  &src );

    /**
      * \return true on sucessful conversion, false on any error.
      * \sa ros2mrpt
      */
    bool mrpt2ros (
        const mrpt::slam::COccupancyGridMap2D &src,
        const std_msgs::Header &header,
        nav_msgs::OccupancyGrid &msg
    );
};


}; //namespace mrpt_bridge

#endif //MRPT_BRIDGE_MAP_H
