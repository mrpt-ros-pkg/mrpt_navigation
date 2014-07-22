
#include "mrpt_bridge/map.h"
#include "mrpt_bridge/pose.h"
#include <mrpt/slam/COccupancyGridMap2D.h>
#include <ros/console.h>
#include <mrpt/base.h>
#include <mrpt/slam.h>

#define INT8_MAX    0x7f
#define INT8_MIN    (-INT8_MAX - 1)
#define INT16_MAX   0x7fff
#define INT16_MIN   (-INT16_MAX - 1)

namespace mrpt_bridge
{
map* map::instance_ = NULL;

map::map ()
{
    /// ceation of the lookup table and pointers
    mrpt::slam::CLogOddsGridMapLUT<mrpt::slam::COccupancyGridMap2D::cellType> table;
#ifdef  OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
    lut_mrpt2rosPtr = lut_mrpt2ros + INT8_MAX + 1; // center the pointer
    for ( int i = INT8_MIN; i < INT8_MAX; i++ ) {
#else
    lut_mrpt2rosPtr = lut_mrpt2ros + INT16_MAX + 1; // center the pointer
    for ( int i = INT16_MIN; INT16_MIN < INT16_MAX; i++ ) {
#endif
        float v = 1.0-table.l2p ( i );
        int idx = v * 100.;
        lut_mrpt2rosPtr[i] = idx;
        //printf("%4i, %4.3f, %4i\n", i, v, idx);
    }
}
map::~map () { }

map* map::instance ()
{
    if ( instance_ == NULL ) instance_ = new map ();
    return instance_;
}

bool map::ros2mrpt ( const nav_msgs::OccupancyGrid  &src, mrpt::slam::COccupancyGridMap2D  &des )
{
    if((src.info.origin.orientation.x != 0) ||
            (src.info.origin.orientation.y != 0) ||
            (src.info.origin.orientation.z != 0) ||
            (src.info.origin.orientation.w != 1)) {
        std::cerr << "Rotated maps are not supported by mrpt!" << std::endl;
        return false;
    }
    float xmin = src.info.origin.position.x;
    float ymin = src.info.origin.position.y;
    float xmax = xmin + src.info.width * src.info.resolution;
    float ymax = ymin + src.info.height * src.info.resolution;

    MRPT_START
    des.setSize(xmin, xmax, ymin, ymax, src.info.resolution);
    MRPT_END
       
    /// I hope the data is allways aligned
    for ( int h = 0; h < src.info.height; h++ ) {
        mrpt::slam::COccupancyGridMap2D::cellType *pDes = des.getRow (h);
        const int8_t *pSrc = &src.data[h * src.info.width];
        for ( int w = 0; w < src.info.width; w++ ) {
            if(*pSrc == -1) {
                *pDes = mrpt::slam::COccupancyGridMap2D::p2l(0.5);
            } else if(*pSrc > 100){         
                std::cerr << "A ros map entry is above 100!, alowed values are [-1,0...100]" << std::endl;
                return false;
            } else {       
                *pDes = mrpt::slam::COccupancyGridMap2D::p2l(1.0 - ((float)*pSrc) /100.0);
            }
            pSrc++;
            pDes++;
        }
    }
    return true;
}
bool map::mrpt2ros (
    const mrpt::slam::COccupancyGridMap2D &src,
    nav_msgs::OccupancyGrid &des,
    const std_msgs::Header &header
)
{
    des.header = header;
    return mrpt2ros(src, des);
}

bool map::mrpt2ros (
    const mrpt::slam::COccupancyGridMap2D &src,
    nav_msgs::OccupancyGrid &des
)
{
    //printf("--------mrpt2ros:  %f, %f, %f, %f, r:%f\n",src.getXMin(), src.getXMax(), src.getYMin(), src.getYMax(), src.getResolution());
    des.info.width = src.getSizeX();
    des.info.height = src.getSizeY();
    des.info.resolution = src.getResolution ();

    des.info.origin.position.x = src.getXMin();
    des.info.origin.position.y = src.getYMin();
    des.info.origin.position.z = 0;

    des.info.origin.orientation.x = 0;
    des.info.origin.orientation.y = 0;
    des.info.origin.orientation.z = 0;
    des.info.origin.orientation.w = 1;

    /// I hope the data is allways aligned
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

const bool map::loadMap(mrpt::slam::CMultiMetricMap &_metric_map, const mrpt::utils::CConfigFile &_config_file, const std::string &_map_file, const std::string &_section_name, bool _debug) {

    mrpt::slam::TSetOfMetricMapInitializers mapInitializers;
    mapInitializers.loadFromConfigFile( _config_file, _section_name);
    if(_debug) mapInitializers.dumpToConsole();

    mrpt::slam::CSimpleMap  simpleMap;

    // Load the set of metric maps to consider in the experiments:
    _metric_map.setListOfMaps( &mapInitializers );
    if(_debug) mapInitializers.dumpToConsole();

    mrpt::random::randomGenerator.randomize();

    // Load the map (if any):
    if (_map_file.empty()) {
        return false;
    } else {
        ASSERT_( mrpt::utils::fileExists(_map_file) );

        // Detect file extension:
        std::string mapExt = mrpt::utils::lowerCase( mrpt::utils::extractFileExtension( _map_file, true ) ); // Ignore possible .gz extensions

        if ( !mapExt.compare( "simplemap" ) )
        {
            // It's a ".simplemap":
            if(_debug) ("Loading '.simplemap' file...");
            mrpt::utils::CFileGZInputStream(_map_file) >> simpleMap;
            printf("Ok\n");

            ASSERT_( simpleMap.size()>0 );

            // Build metric map:
            if(_debug) printf("Building metric map(s) from '.simplemap'...");
            _metric_map.loadFromProbabilisticPosesAndObservations(simpleMap);
            if(_debug) printf("Ok\n");
        }
        else if ( !mapExt.compare( "gridmap" ) )
        {
            // It's a ".gridmap":
            if(_debug) printf("Loading gridmap from '.gridmap'...");
            ASSERT_( _metric_map.m_gridMaps.size()==1 );
            mrpt::utils::CFileGZInputStream(_map_file) >> (*_metric_map.m_gridMaps[0]);
            if(_debug) printf("Ok\n");
        }
        else
        {
            THROW_EXCEPTION_CUSTOM_MSG1("Map file has unknown extension: '%s'",mapExt.c_str());
            return false;
        }

    }
    return true;
}
} // end namespace