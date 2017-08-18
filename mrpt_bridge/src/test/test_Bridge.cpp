//
// Created by raghavender on 17/08/17.
//

#include "mrpt_bridge/GPS.h"
#include "ros/ros.h"

/// mrpt imports
#include <dirent.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/utils/metaprogramming.h>
#include <mrpt/math/data_utils.h>
#include <mrpt/vision/tracking.h>
#include <mrpt/system/os.h>
#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CAction.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CRawlog.h>


#include <boost/interprocess/sync/scoped_lock.hpp>
#include <iostream>
#include <memory>
#include <mrpt/version.h>

///c++ standard stuff
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <boost/bind.hpp>

/// opencv includes
#include "opencv2/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt;
using namespace mrpt::poses;

using namespace std;
using namespace cv;


int rawlog_type = 0;

string getImageDir(string path)
{
    long i = path.size()-1;
    for(; i>=0 ; i--)
    {
        if(path.at(i) == '/')
        {
            break;
        }
    }
    string temp = path.substr(0,i);;
    return temp;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;


    /// APPROACH 2
    string rawlog = "/home/raghavender/Desktop/GSoC/datasets/malaga-urban-dataset-extract-01/malaga-urban-dataset-extract-01_all-sensors.rawlog";
    CRawlog dataset;
    dataset.loadFromRawLogFile(rawlog);
    cout << dataset.size() << " entries loaded." << endl;


    string errorMsg;
    ofstream ostream1;

    for(unsigned int i=0 ; i<dataset.size() ; i++)
    {
        try
        {
            switch (dataset.getType(i))
            {
                case CRawlog::etObservation:
                {
                    cout << "etObservation " << i << endl;
                    CObservation::Ptr o = dataset.getAsObservation(i);
                    //cout << o->sensorLabel << " type : etObs" << dataset.getType(i) << " index: " << i << endl;

                    if (IS_CLASS(o, CObservationStereoImages))
                    {
                        cout << " stereo Image detected " << endl;
                        CObservationStereoImages::Ptr obsSt = std::dynamic_pointer_cast<CObservationStereoImages>(o);

                        ///similar to common practice of reading images-->  https://github.com/MRPT/mrpt/search?utf8=%E2%9C%93&q=IMAGES_PATH_BASE&type=
                        stringstream str;
                        str << getImageDir(rawlog) << "/Images";
                        obsSt->imageLeft.IMAGES_PATH_BASE = str.str();

                        cout << obsSt->imageLeft.getExternalStorageFileAbsolutePath() << " external " << endl;

                        rawlog_type = 1;

                        CImage image_c = obsSt->imageLeft;
                        Mat cvImg1 = cv::cvarrToMat(image_c.getAs<IplImage>());
                        //imshow("view", cvImg1);
                        //waitKey(1);
                    }
                    else if (IS_CLASS(o, CObservationImage))
                    {
                        cout << "monocular image detected " << endl;
                        CObservationImage::Ptr obsIm = std::dynamic_pointer_cast<CObservationImage>(o);

                        ///similar to common practice of reading images-->  https://github.com/MRPT/mrpt/search?utf8=%E2%9C%93&q=IMAGES_PATH_BASE&type=
                        stringstream str;
                        str << getImageDir(rawlog) << "/Images";
                        obsIm->image.IMAGES_PATH_BASE = str.str();

                    }
                    else if (IS_CLASS(o, CObservationGPS))
                    {
                        /// GPS reading and publishing happens here
                        CObservationGPS::Ptr obsGPS = std::dynamic_pointer_cast<CObservationGPS>(o);

                    }
                    else if(IS_CLASS(o, CObservationIMU))
                    {
                        /// IMU publishing of messages happens here
                        CObservationIMU::Ptr obsImu = std::dynamic_pointer_cast<CObservationIMU>(o);
                    }
                    else if(IS_CLASS(o, CObservationRange))
                    {
                        /// Range publishing of messages happens here

                        CObservationRange::Ptr obsRng = std::dynamic_pointer_cast<CObservationRange>(o);
                    }
                }break;
                case CRawlog::etActionCollection:
                {
                    cout << "etActionCollection " << i << endl;
                    cout << " type : etAC " << dataset.getType(i) << " index: " << i << endl;
                    break;
                }
                default:
                    cout << " I am in default block" << endl;
            }



        }// end of the try block
        catch (exception& e)
        {
            cout << " CATCH CATCH CATCH BLOCK %$#@%($@*%(&@(%&@(%@(#$%(@#^$%" << endl;
            errorMsg = e.what();
            break;
        }

    } // end of for loop

    while(ros::ok())
    {
        ROS_INFO("iNSIDE WHILE \n");
        ros::spinOnce();
    }

    cout << " Hello" << endl;
    return 0;
}


