
#include "mrpt_bridge/map.h"
#include <mrpt/slam/COccupancyGridMap2D.h>
#include <ros/console.h>

class COccupancyGridMap2DBridge : public mrpt::slam::COccupancyGridMap2D
{
public:
    const std::vector<mrpt::slam::COccupancyGridMap2D::cellType>  getData() const {
        return this->map;
    }
};

namespace mrpt_bridge
{
map* map::instance_ = NULL;

map::map ()
{
    /*
#ifdef  OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
    for ( int8_t i = 0; i < 0xFF; i++ ) {
#else
    for ( int16_t i = 0; i < 0xFFFF; i++ ) {
#endif
        lut_mrpt2ros[i] = ( 1.0-mrpt::slam::COccupancyGridMap2D::l2p ( i ) ) *100;
    }
    */
}
map::~map () { }

map* map::instance ()
{
    if ( !instance_ ) instance_ = new map ();
    return instance_;
}

bool map::ros2mrpt ( const mrpt::slam::COccupancyGridMap2D  &src, nav_msgs::OccupancyGrid  &des )
{
    MRPT_TODO ( "ros2mrpt for COccupancyGridMap2D" );
    ROS_ERROR( "ros2mrpt for COccupancyGridMap2D not implemented");
    return false;
}
bool map::mrpt2ros (
    const mrpt::slam::COccupancyGridMap2D &src,
    const std_msgs::Header &header,
    const geometry_msgs::Pose &pose,
    nav_msgs::OccupancyGrid &des
)
{
    COccupancyGridMap2DBridge &mrptMap = ( COccupancyGridMap2DBridge & ) src;
    des.header = header;
    des.info.width = mrptMap.getSizeX();
    des.info.height = mrptMap.getSizeY();
    des.info.resolution = mrptMap.getResolution ();
    des.info.origin = pose;

    const std::vector<mrpt::slam::COccupancyGridMap2D::cellType> &srcMap = mrptMap.getData();
    des.data.resize ( srcMap.size() );
    for ( size_t i = 0; i < srcMap.size(); i++ ) {
        //des.data[i] = lut_mrpt2ros[map[i]];
        des.data[i]  = ( 1.0-src.l2p ( srcMap[i] ) ) *100;
    }
    return true;
}
} // end namespace
