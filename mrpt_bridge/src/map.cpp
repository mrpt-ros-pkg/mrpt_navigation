
#include "mrpt_bridge/map.h"
#include <mrpt/slam/COccupancyGridMap2D.h>
#include <ros/console.h>

#define INT8_MAX    0x7f
#define INT8_MIN    (-INT8_MAX - 1)
#define INT16_MAX   0x7fff
#define INT16_MIN   (-INT16_MAX - 1)

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
    mrpt::slam::CLogOddsGridMapLUT<mrpt::slam::COccupancyGridMap2D::cellType> table;
#ifdef  OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
    lut_mrpt2rosPtr = lut_mrpt2ros + INT8_MAX + 1; // center the pointer
    for ( int i = INT8_MIN; i < INT8_MAX; i++ ) {
#else
    lut_mrpt2rosPtr = lut_mrpt2ros + INT16_MAX + 1; // center the pointer
    for ( int i = INT16_MIN; INT16_MIN < INT16_MAX; i++ ) {
#endif
        lut_mrpt2rosPtr[i] = ( 1.0-table.l2p ( i )) * 100;
    }
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

    //const std::vector<mrpt::slam::COccupancyGridMap2D::cellType> &srcData = mrptMap.getData();
    des.data.resize ( des.info.width*des.info.height );
    for ( int h = 0; h < des.info.height; h++ ) {
        const mrpt::slam::COccupancyGridMap2D::cellType *pSrc = src.getRow (h);
        int8_t *pDes = &des.data[h * des.info.width];
        for ( int w = 0; w < des.info.width; w++ ) {
            *pDes++ = lut_mrpt2rosPtr[*pSrc++];
        }
    }
    return true;
}
} // end namespace
