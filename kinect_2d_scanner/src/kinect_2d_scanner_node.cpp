
#include <driver_base/driver.h>
#include <driver_base/driver_node.h>
#include <diagnostic_updater/publisher.h>

#include <assert.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <iomanip>

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

#include "kinect_2d_scanner/TKinect2DScannerConfig.h"

// Conversions ROS msgs <-> MRPT structs
#include <mrpt_bridge/mrpt_bridge.h>

// MRPT stuff:
#include <mrpt/hwdrivers/CKinect.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/system/threads.h>
#include <mrpt/synch.h>
#include <mrpt/gui.h>
#include <mrpt/opengl.h>
#include <mrpt/opengl/CPlanarLaserScan.h>

#include <mrpt/version.h>
#if MRPT_VERSION < 0x096
#	error "This program requires MRPT >= 0.9.6"
#endif


using namespace std;
using namespace mrpt::slam;  // for mrpt::CObservation*

// Thread for grabbing: Do this is another thread to exploit multicore CPUs.
struct TThreadParam
{
	typedef kinect_2d_scanner::TKinect2DScannerConfig  TConfig;

	TThreadParam(TConfig  &_config) : quit(false), config(_config)  { }

	volatile bool   quit;

	volatile TConfig  &config; //!< Dynamic reconfigure set of node configs

	mrpt::synch::CThreadSafeVariable<CObservation3DRangeScanPtr> new_obs;     // RGB+D (+3D points)
	mrpt::synch::CThreadSafeVariable<CObservationIMUPtr>         new_obs_imu; // Accelerometers

};

void thread_grabbing(TThreadParam &p)
{
	try
	{
		mrpt::hwdrivers::CKinect  kinect;

		bool there_is_obs=true, hard_error=false;

		while (!hard_error && !p.quit)
		{
			// Is kinect already open? If not open now:
			if (!kinect.isOpen())
			{
				kinect.enableGrab3DPoints(true);
				kinect.enableGrabRGB(true);
				kinect.enableGrabDepth(true);

				ROS_INFO("Calling CKinect::initialize()...");
				kinect.initialize();
				ROS_INFO("Kinect sensor opened OK!");
			}
			else
			{
				// Grab new observation from the camera:
				CObservation3DRangeScanPtr  obs     = CObservation3DRangeScan::Create(); // Smart pointers to observations
				CObservationIMUPtr          obs_imu = CObservationIMU::Create();

				// This call doesn't lock CPU: puts the thread in sleep for tiny
				// intervals until a new obs arrives or it timeouts:
				kinect.getNextObservation(*obs,*obs_imu,there_is_obs,hard_error);

				if (!hard_error && there_is_obs)
				{
					p.new_obs.set(obs);
					p.new_obs_imu.set(obs_imu);
				}
			}
		} // end while loop

		// Kinect is closed automatically on destruction
	}
	catch(std::exception &e)
	{
		ROS_ERROR("Unexpected exception in Kinect thread: %s", e.what());
		p.quit = true;
	}
}

/** All the data associated to the (optional) live GUI of this node */
struct TGUIData
{
	mrpt::gui::CDisplayWindow3DPtr  win3D;  // Optional gui:

	mrpt::opengl::CPointCloudColouredPtr gl_points; //!< The 3D point cloud OpenGL object
	mrpt::opengl::CPlanarLaserScanPtr gl_2d_scan;   //!< The 2D "laser scan" OpenGL object
	mrpt::opengl::CFrustumPtr gl_frustum;
	mrpt::opengl::COpenGLViewportPtr  viewInt; //!< Extra viewports for the RGB images.


	TGUIData(const kinect_2d_scanner::TKinect2DScannerConfig &_config)
	{
		updateFromConfig(_config);
	}

	void updateFromConfig(const kinect_2d_scanner::TKinect2DScannerConfig &_config);

private:
	void start();
};

// All callbacks requird when dynamic reconfigure sends us something:
void reconfigure_all(
	kinect_2d_scanner::TKinect2DScannerConfig &new_config,
	uint32_t level,
	kinect_2d_scanner::TKinect2DScannerConfig *config_placeholder,
	TGUIData &gui_data
	)
{
	//ROS_INFO("Reconfigure called at level %x.", level);

	// The target object was copied by reference in all other places when it's needed.
	*config_placeholder = new_config;

	gui_data.updateFromConfig( *config_placeholder );
}


/** Simple first-order low-pass filter estimator of event rates. */
class CSimpleRateEstimator
{
public:
	CSimpleRateEstimator(const double smooth_factor = 0.99) :
		m_count(),
		m_rate_est(0),
		m_alpha(smooth_factor),
		m_alpha_comp(1.0-smooth_factor)
	{
		m_last_tim = m_tictac.Tac();
	}

	/** Do a 'tick', that is, signal that the event whatever just happened */
	void tick()
	{
		const double t = m_tictac.Tac();
		const double At = std::max(1e-10, t-m_last_tim);
		const double freq = 1./At;
		m_rate_est = m_alpha*m_rate_est + m_alpha_comp*freq;
		m_last_tim=t;
	}

	/** Retrieve rate estimate (in Hz) */
	double getEstimatedRate() const { return m_rate_est; }

private:
	int      m_count;
	double   m_rate_est;
	double   m_last_tim;
	double   m_alpha, m_alpha_comp;
	mrpt::utils::CTicTac  m_tictac;
};


/** The main */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "kinect_2d_scanner_node");

	// ROS comms entry point:
	// -----------------------------------------
	ros::NodeHandle node_handle;
	ros::Publisher laser_pub = node_handle.advertise<sensor_msgs::LaserScan>("scan", 100 /* max buffer */ );

	ros::Rate loop_rate(200);  // Must be >= real sensor FPS (in Kinect=30Hz)

	// Sensor Diagnostics:
	// -----------------------------------------
#if 0
	diagnostic_updater::Updater diagnostic (...);
	diagnostic_updater::DiagnosedPublisher<sensor_msgs::LaserScan> scan_pub(
		node_handle.advertise<sensor_msgs::LaserScan>("scan", 100),
        diagnostic_,
        diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05),
        diagnostic_updater::TimeStampStatusParam())
	diagnostic_updater::FunctionDiagnosticTask  my_diagnostic_task(
		"Kinect-to-2D Diagnostics", boost::bind(&HokuyoNode::connectionStatus, this, _1))
#endif


	// Set of node parameters (for dynamic reconfigure):
	// ---------------------------------------------------
	kinect_2d_scanner::TKinect2DScannerConfig  config = kinect_2d_scanner::TKinect2DScannerConfig::__getDefault__();

	// Aux data structs:
	TGUIData     gui_data(config);
	TThreadParam thrPar  (config);

	// Launch grabbing thread:
	// ---------------------------------------------------
	mrpt::system::TThreadHandle grab_thread_handle = mrpt::system::createThreadRef(thread_grabbing ,thrPar);

	// Attach reconfigure server:
	// ---------------------------------------------------
	dynamic_reconfigure::Server<kinect_2d_scanner::TKinect2DScannerConfig> reconfigure_server( ros::NodeHandle("~") );

    dynamic_reconfigure::Server<kinect_2d_scanner::TKinect2DScannerConfig>::CallbackType f
		= boost::bind(&reconfigure_all,_1,_2, &config,gui_data);

    reconfigure_server.setCallback(f);


	// Main loop
	// -------------------------------------------------------------
	CObservation3DRangeScanPtr  last_obs;
	CObservationIMUPtr          last_obs_imu;

	CSimpleRateEstimator        rate_estimate;
	int  verbose_out_count = 0;

	while (ros::ok() && !thrPar.quit)
	{
		CObservation3DRangeScanPtr possiblyNewObs = thrPar.new_obs.get();
		if (possiblyNewObs && possiblyNewObs->timestamp!=INVALID_TIMESTAMP &&
			(!last_obs  || possiblyNewObs->timestamp!=last_obs->timestamp ) )
		{
			// It IS a new observation:
			last_obs     = possiblyNewObs;
			last_obs_imu = thrPar.new_obs_imu.get();

			// Convert to equivalent 2D laser scan:
			mrpt::slam::CObservation2DRangeScan scan2d;

			last_obs->convertTo2DScan(
				scan2d,
				"KINECT_2D",   // MRPT's SensorLabel
				DEG2RAD( config.vert_half_FOV_up ),
				DEG2RAD( config.vert_half_FOV_low ),
				config.oversampling_ratio );

			// MRPT -> ROS:
			sensor_msgs::LaserScan scan_msg;

			if (mrpt_bridge::convert(scan2d,scan_msg))
			{
				// Publish:
				laser_pub.publish(scan_msg);
				rate_estimate.tick();
			}
			// else // do warning?

			// other GUI stuff:
			if (config.show_GUI && gui_data.win3D && gui_data.win3D->isOpen())
			{
				bool do_refresh = false;

				gui_data.win3D->get3DSceneAndLock();
				// Show intensity image:
				if (last_obs->hasIntensityImage)
				{
					gui_data.viewInt->setImageView(last_obs->intensityImage); // This is not "_fast" since the intensity image is used below in the coloured point cloud.
					do_refresh=true;
				}

				// Show 3D points:
				if (last_obs->hasPoints3D)
				{
					last_obs->project3DPointsFromDepthImageInto(*gui_data.gl_points, false /* ignore pose_on_robot of the sensor */ );
					do_refresh=true;
				}

				// Set 2d scan in OpenGL object for rendering:
				gui_data.gl_2d_scan->setScan(scan2d);


				gui_data.win3D->addTextMessage(
					10,10,
					mrpt::format("Sensor rate: %.02fHz",rate_estimate.getEstimatedRate()),
					TColorf(1,1,1),"sans",10, mrpt::opengl::FILL, 101 /* unique label ID */ );

				gui_data.win3D->unlockAccess3DScene();

				if (do_refresh) gui_data.win3D->repaint();
			} // end update gui

		}

		// Some console output to let user know I'm alive:
		if (++verbose_out_count>400) {
			verbose_out_count=0;
			ROS_INFO("Kinect-to-2D rate: %.02fHz",rate_estimate.getEstimatedRate());
		}

		// End loop:
		ros::spinOnce();
		loop_rate.sleep();
	}

	cout << "Waiting for grabbing thread to exit...\n";
	thrPar.quit = true;
	mrpt::system::joinThread(grab_thread_handle);
	cout << "Bye!";

	return 0;
}


// ------------ GUI stuff ------------
void TGUIData::start()
{
	win3D = mrpt::gui::CDisplayWindow3D::Create("kinect_2d_scanner_node GUI", 640,480);

	win3D->setCameraAzimuthDeg(140);
	win3D->setCameraElevationDeg(30);
	win3D->setCameraZoom(10.0);
	win3D->setFOV(90);
	win3D->setCameraPointingToPoint(2.5,0,0);

	// The 3D point cloud OpenGL object:
	gl_points = mrpt::opengl::CPointCloudColoured::Create();
	gl_points->setPointSize(2.5);

	// The 2D "laser scan" OpenGL object:
	gl_2d_scan = mrpt::opengl::CPlanarLaserScan::Create();
	gl_2d_scan->enablePoints(true);
	gl_2d_scan->enableLine(true);
	gl_2d_scan->enableSurface(true);
	gl_2d_scan->setSurfaceColor(0,0,1, 0.3);  // RGBA

	gl_frustum = mrpt::opengl::CFrustum::Create(0.2f, 5.0f, 90.0f, 5.0f, 2.0f, true, true );

	const double aspect_ratio =  480.0 / 640.0; // kinect.getRowCount() / double( kinect.getColCount() );

	{
		mrpt::opengl::COpenGLScenePtr &scene = win3D->get3DSceneAndLock();

		// Create the Opengl object for the point cloud:
		scene->insert( gl_points );
		scene->insert( gl_2d_scan );
		scene->insert( gl_frustum );

		{
			mrpt::opengl::CGridPlaneXYPtr gl_grid = mrpt::opengl::CGridPlaneXY::Create();
			gl_grid->setColor(0.6,0.6,0.6);
			scene->insert( gl_grid );
		}
		{
			mrpt::opengl::CSetOfObjectsPtr gl_corner = mrpt::opengl::stock_objects::CornerXYZ();
			gl_corner->setScale(0.2);
			scene->insert(gl_corner);
		}

		const int VW_WIDTH = 250;	// Size of the viewport into the window, in pixel units.
		const int VW_HEIGHT = aspect_ratio*VW_WIDTH;
		const int VW_GAP = 30;

		// Create the Opengl objects for the planar images, as textured planes, each in a separate viewport:
		win3D->addTextMessage(30, -25-1*(VW_GAP+VW_HEIGHT),"Intensity data",TColorf(1,1,1), 2, MRPT_GLUT_BITMAP_HELVETICA_12 );
		viewInt = scene->createViewport("view2d_int");
		viewInt->setViewportPosition(5, -10-1*(VW_GAP+VW_HEIGHT), VW_WIDTH,VW_HEIGHT );

		win3D->unlockAccess3DScene();
		win3D->repaint();
	}

}

void TGUIData::updateFromConfig(const kinect_2d_scanner::TKinect2DScannerConfig &config)
{
	// Start/stop:
	if (config.show_GUI && !win3D) start();
	else if (!config.show_GUI && win3D && win3D->isOpen())
	{
		// Close window & destroy object.
		win3D.clear();
	}

	if (!config.show_GUI) return; // Nothing else to do.

	// Normal update of params:
	// -------------------------
	if (gl_frustum)
	{
		gl_frustum->setVisibility( config.show_frustum );
		gl_frustum->setVertFOVAsymmetric  ( config.vert_half_FOV_low, config.vert_half_FOV_up );
	}
	if (gl_2d_scan)
	{
		gl_2d_scan->setVisibility( config.show_2d_scan );
	}
	if (gl_points)
	{
		gl_points->setVisibility( config.show_3d_points );
	}

	win3D->repaint();
}
