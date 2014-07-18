
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
#ifdef  OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
    mrpt::slam::CLogOddsGridMapLUT<mrpt::slam::COccupancyGridMap2D::cellType> table;
    printf(" --------------------------- COccupancyGridMap2D  cellType = int8_t \n");
    for ( unsigned int i = 0; i < 0xFF; i++ ) {
#else
    mrpt::slam::CLogOddsGridMapLUT<mrpt::slam::COccupancyGridMap2D::cellType> table;
    printf(" --------------------------- COccupancyGridMap2D  cellType = int16_t \n");
    for ( unsigned int i = 0; i < 0xFFFF; i++ ) {
#endif
        MRPT_TODO ( "Markus Bader: This is not working and I don't know " );
        lut_mrpt2ros[i] = ( 1.0-table.l2p ( i ) ) *100;
    }
    fflush(stdout);
}
map::~map () { }

map* map::instance ()
{
    if ( instance_ == NULL ) instance_ = new map ();
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

    const std::vector<mrpt::slam::COccupancyGridMap2D::cellType> &srcData = mrptMap.getData();
    des.data.resize ( srcData.size() );
    for ( size_t i = 0; i < srcData.size(); i++ ) {
        MRPT_TODO ( "Markus Bader: The lut table would be faster " );
        //des.data[i] = lut_mrpt2ros[srcData[i]];
        des.data[i]  = ( 1.0-src.l2p ( srcData[i] ) ) *100;
    }
    return true;
}
} // end namespace
