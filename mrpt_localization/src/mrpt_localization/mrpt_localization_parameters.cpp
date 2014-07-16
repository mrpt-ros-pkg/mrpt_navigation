/***********************************************************************************
 * Revised BSD License                                                             *
 * Copyright (c) 2014, Markus Bader <markus.bader@tuwien.ac.at>                    *
 * All rights reserved.                                                            *
 *                                                                                 *
 * Redistribution and use in source and binary forms, with or without              *
 * modification, are permitted provided that the following conditions are met:     *
 *     * Redistributions of source code must retain the above copyright            *
 *       notice, this list of conditions and the following disclaimer.             *
 *     * Redistributions in binary form must reproduce the above copyright         *
 *       notice, this list of conditions and the following disclaimer in the       *
 *       documentation and/or other materials provided with the distribution.      *
 *     * Neither the name of the Vienna University of Technology nor the           *
 *       names of its contributors may be used to endorse or promote products      *
 *       derived from this software without specific prior written permission.     *
 *                                                                                 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND *
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED   *
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE          *
 * DISCLAIMED. IN NO EVENT SHALL Markus Bader BE LIABLE FOR ANY                    *
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES      *
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;    *
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND     *
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT      *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS   *
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                    *                       *
 ***********************************************************************************/


#include <mrpt_localization/mrpt_localization.h>
#include <mrpt_localization/mrpt_localization_defaults.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace mrpt::random;
using namespace std;


PFLocalization::Parameters::Parameters()
    : debug(MRPT_LOCALIZATION_DEFAULT_DEBUG)
    , iniFile(MRPT_LOCALIZATION_DEFAULT_INI_FILE)
    , rawlogFile(MRPT_LOCALIZATION_DEFAULT_RAWLOG_FILE)
    , mapFile(MRPT_LOCALIZATION_DEFAULT_MAP_FILE) {
}

void PFLocalization::init() {
    printf("iniFile ready %s\n", param_->iniFile.c_str());
    ASSERT_FILE_EXISTS_(param_->iniFile)
            printf("ASSERT_FILE_EXISTS_ %s\n", param_->iniFile.c_str());
    iniFile_.setFileName(param_->iniFile);
    printf("CConfigFile %s\n", param_->iniFile.c_str());


    vector_int          particles_count;    // Number of initial particles (if size>1, run the experiments N times)

    // Load configuration:
    // -----------------------------------------
    string iniSectionName ( "LocalizationExperiment" );


    // Mandatory entries:
    iniFile_.read_vector(iniSectionName, "particles_count", vector_int(1,0), particles_count, /*Fail if not found*/true );
    string      OUT_DIR_PREFIX      = iniFile_.read_string(iniSectionName,"logOutput_dir","", /*Fail if not found*/true );


    string      RAWLOG_FILE;
    if (param_->rawlogFile.empty())
        RAWLOG_FILE = iniFile_.read_string(iniSectionName,"rawlog_file","", /*Fail if not found*/true );
    else RAWLOG_FILE = param_->rawlogFile;

    string      MAP_FILE;
    if (param_->mapFile.empty())
        MAP_FILE = iniFile_.read_string(iniSectionName,"map_file","" );
    else MAP_FILE = param_->mapFile;

    // Non-mandatory entries:
    size_t      rawlog_offset       = iniFile_.read_int(iniSectionName,"rawlog_offset",0);
    string      GT_FILE             = iniFile_.read_string(iniSectionName,"ground_truth_path_file","");
    NUM_REPS_          = iniFile_.read_int(iniSectionName,"experimentRepetitions",1);
    SCENE3D_FREQ_        = iniFile_.read_int(iniSectionName,"3DSceneFrequency",10);
    SCENE3D_FOLLOW_ = iniFile_.read_bool(iniSectionName,"3DSceneFollowRobot",true);
    testConvergenceAt_   = iniFile_.read_int(iniSectionName,"experimentTestConvergenceAtStep",-1);

    SAVE_STATS_ONLY_ = iniFile_.read_bool(iniSectionName,"SAVE_STATS_ONLY",false);
    SHOW_PROGRESS_3D_REAL_TIME_ = iniFile_.read_bool(iniSectionName,"SHOW_PROGRESS_3D_REAL_TIME",false);
    SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS_ = iniFile_.read_int(iniSectionName,"SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS",1);
    STATS_CONF_INTERVAL_ = iniFile_.read_double(iniSectionName,"STATS_CONF_INTERVAL",0.2);

#if !MRPT_HAS_WXWIDGETS
    SHOW_PROGRESS_3D_REAL_TIME = false;
#endif

    // Default odometry uncertainty parameters in "dummy_odom_params" depending on how fast the robot moves, etc...
    //  Only used for observations-only rawlogs:

    dummy_odom_params_.modelSelection = CActionRobotMovement2D::mmGaussian;
    dummy_odom_params_.gausianModel.minStdXY  = iniFile_.read_double("DummyOdometryParams","minStdXY",0.04);
    dummy_odom_params_.gausianModel.minStdPHI = DEG2RAD(iniFile_.read_double("DummyOdometryParams","minStdPHI", 2.0));


    // PF-algorithm Options:
    // ---------------------------
    CParticleFilter::TParticleFilterOptions     pfOptions;
    pfOptions.loadFromConfigFile( iniFile_, "PF_options" );

    // PDF Options:
    // ------------------
    TMonteCarloLocalizationParams   pdfPredictionOptions;
    pdfPredictionOptions.KLD_params.loadFromConfigFile( iniFile_, "KLD_options");

    // Metric map options:
    // -----------------------------
    TSetOfMetricMapInitializers             mapList;
    mapList.loadFromConfigFile( iniFile_,"metricMap");



    cout<< "-------------------------------------------------------------\n"
        << "\t RAWLOG_FILE = \t "   << RAWLOG_FILE << endl
        << "\t MAP_FILE = \t "      << MAP_FILE << endl
        << "\t GT_FILE = \t "       << GT_FILE << endl
        << "\t OUT_DIR_PREFIX = \t "<< OUT_DIR_PREFIX << endl
        << "\t #particles = \t "    << particles_count << endl
        << "-------------------------------------------------------------\n";
    pfOptions.dumpToConsole();
    mapList.dumpToConsole();

    // --------------------------------------------------------------------
    //                      EXPERIMENT PREPARATION
    // --------------------------------------------------------------------
    CSimpleMap  simpleMap;

    // Load the set of metric maps to consider in the experiments:
    metricMap_.setListOfMaps( &mapList );
    mapList.dumpToConsole();

    randomGenerator.randomize();

    // Load the map (if any):
    // -------------------------
    if (MAP_FILE.size())
    {
        ASSERT_( fileExists(MAP_FILE) );

        // Detect file extension:
        // -----------------------------
        string mapExt = lowerCase( extractFileExtension( MAP_FILE, true ) ); // Ignore possible .gz extensions

        if ( !mapExt.compare( "simplemap" ) )
        {
            // It's a ".simplemap":
            // -------------------------
            printf("Loading '.simplemap' file...");
            CFileGZInputStream(MAP_FILE) >> simpleMap;
            printf("Ok\n");

            ASSERT_( simpleMap.size()>0 );

            // Build metric map:
            // ------------------------------
            printf("Building metric map(s) from '.simplemap'...");
            metricMap_.loadFromProbabilisticPosesAndObservations(simpleMap);
            printf("Ok\n");
        }
        else if ( !mapExt.compare( "gridmap" ) )
        {
            // It's a ".gridmap":
            // -------------------------
            printf("Loading gridmap from '.gridmap'...");
            ASSERT_( metricMap_.m_gridMaps.size()==1 );
            CFileGZInputStream(MAP_FILE) >> (*metricMap_.m_gridMaps[0]);
            printf("Ok\n");
        }
        else
        {
            THROW_EXCEPTION_CUSTOM_MSG1("Map file has unknown extension: '%s'",mapExt.c_str());
        }

    }

    // Load the Ground Truth:
    groundTruth_ = CMatrixDouble(0,0);
    if ( fileExists( GT_FILE ) )
    {
        printf("Loading ground truth file...");
        groundTruth_.loadFromTextFile( GT_FILE );
        printf("OK\n");
    }
    else
        printf("Ground truth file: NO\n");


    // Create 3D window if requested:
    if (SHOW_PROGRESS_3D_REAL_TIME_)
    {
        win3D_ = CDisplayWindow3D::Create("pf-localization - The MRPT project", 1000, 600);
        win3D_->setCameraZoom(20);
        win3D_->setCameraAzimuthDeg(-45);
        //win3D_->waitForKey();
    }

    // Create the 3D scene and get the map only once, later we'll modify only the particles, etc..

    // The gridmap:
    if (metricMap_.m_gridMaps.size())
    {
        metricMap_.m_gridMaps[0]->computeEntropy( gridInfo_ );
        printf("The gridmap has %.04fm2 observed area, %u observed cells\n", gridInfo_.effectiveMappedArea, (unsigned) gridInfo_.effectiveMappedCells );

        {
            CSetOfObjectsPtr plane = CSetOfObjects::Create();
            metricMap_.m_gridMaps[0]->getAs3DObject( plane );
            scene_.insert( plane );
        }

        if (SHOW_PROGRESS_3D_REAL_TIME_)
        {
            COpenGLScenePtr ptrScene = win3D_->get3DSceneAndLock();

            CSetOfObjectsPtr plane = CSetOfObjects::Create();
            metricMap_.m_gridMaps[0]->getAs3DObject( plane );
            ptrScene->insert( plane );

            ptrScene->enableFollowCamera(true);

            win3D_->unlockAccess3DScene();
        }
    }


    int     PARTICLE_COUNT = *particles_count.begin();

    printf("Initial PDF: %f particles/m2\n", PARTICLE_COUNT/gridInfo_.effectiveMappedArea);


    // Global stats for all the experiment loops:
   nConvergenceTests_ = 0;
   nConvergenceOK_ = 0;
   covergenceErrors_.clear();
    // --------------------------------------------------------------------
    //                  EXPERIMENT REPETITIONS LOOP
    // --------------------------------------------------------------------
    tictacGlobal_.Tic();

    // --------------------------
    // Load the rawlog:
    // --------------------------
    printf("Opening the rawlog file...");
    CFileGZInputStream rawlog_in_stream(RAWLOG_FILE);
    printf("OK\n");

    // The experiment directory is:

    if (!SAVE_STATS_ONLY_)
    {
        sOUT_DIR_        = format("%s",OUT_DIR_PREFIX.c_str());
        sOUT_DIR_PARTS_  = format("%s/particles", sOUT_DIR_.c_str());
        sOUT_DIR_3D_ = format("%s/3D", sOUT_DIR_.c_str());

        printf("Creating directory: %s\n",sOUT_DIR_.c_str());
        createDirectory( sOUT_DIR_ );
        ASSERT_(fileExists(sOUT_DIR_));
        deleteFiles(format("%s/*.*",sOUT_DIR_.c_str()));

        printf("Creating directory: %s\n",sOUT_DIR_PARTS_.c_str());
        createDirectory( sOUT_DIR_PARTS_ );
        ASSERT_(fileExists(sOUT_DIR_PARTS_));
        deleteFiles(format("%s/*.*",sOUT_DIR_PARTS_.c_str()));

        printf("Creating directory: %s\n",sOUT_DIR_3D_.c_str());
        createDirectory( sOUT_DIR_3D_ );
        ASSERT_(fileExists(sOUT_DIR_3D_));
        deleteFiles(format("%s/*.*",sOUT_DIR_3D_.c_str()));

        metricMap_.m_gridMaps[0]->saveAsBitmapFile(format("%s/gridmap.png",sOUT_DIR_.c_str()));
        CFileOutputStream(format("%s/gridmap_limits.txt",sOUT_DIR_.c_str())).printf(
                    "%f %f %f %f",
                    metricMap_.m_gridMaps[0]->getXMin(),metricMap_.m_gridMaps[0]->getXMax(),
                    metricMap_.m_gridMaps[0]->getYMin(),metricMap_.m_gridMaps[0]->getYMax() );

        // Save the landmarks for plot in matlab:
        if (metricMap_.m_landmarksMap)
            metricMap_.m_landmarksMap->saveToMATLABScript2D(format("%s/plot_landmarks_map.m",sOUT_DIR_.c_str()));
    }

    MRPT_TODO("Max to Jose: can I reinizialize the pdf like this, should we add a function or should I use a ptr?");
    pdf_.clear();
    pdf_.derived () = mrpt::poses::CPosePDFParticles(PARTICLE_COUNT);

    // PDF Options:
    pdf_.options = pdfPredictionOptions;

    pdf_.options.metricMap = &metricMap_;

    // Create the PF object:
    pf_.m_options = pfOptions;

    size_t  step = 0;
    size_t rawlogEntry = 0;

    // Initialize the PDF:
    // -----------------------------
    tictac_.Tic();
    if ( !iniFile_.read_bool(iniSectionName,"init_PDF_mode",false, /*Fail if not found*/true) )
        pdf_.resetUniformFreeSpace(
                    metricMap_.m_gridMaps[0].pointer(),
                    0.7f,
                    PARTICLE_COUNT ,
                    iniFile_.read_float(iniSectionName,"init_PDF_min_x",0,true),
                    iniFile_.read_float(iniSectionName,"init_PDF_max_x",0,true),
                    iniFile_.read_float(iniSectionName,"init_PDF_min_y",0,true),
                    iniFile_.read_float(iniSectionName,"init_PDF_max_y",0,true),
                    DEG2RAD(iniFile_.read_float(iniSectionName,"init_PDF_min_phi_deg",-180)),
                    DEG2RAD(iniFile_.read_float(iniSectionName,"init_PDF_max_phi_deg",180))
                    );
    else
        pdf_.resetUniform(
                    iniFile_.read_float(iniSectionName,"init_PDF_min_x",0,true),
                    iniFile_.read_float(iniSectionName,"init_PDF_max_x",0,true),
                    iniFile_.read_float(iniSectionName,"init_PDF_min_y",0,true),
                    iniFile_.read_float(iniSectionName,"init_PDF_max_y",0,true),
                    DEG2RAD(iniFile_.read_float(iniSectionName,"init_PDF_min_phi_deg",-180)),
                    DEG2RAD(iniFile_.read_float(iniSectionName,"init_PDF_max_phi_deg",180)),
                    PARTICLE_COUNT
                    );


    printf("PDF of %u particles initialized in %.03fms\n", PARTICLE_COUNT, 1000*tictac_.Tac());

    // -----------------------------
    //      Particle filter
    // -----------------------------
    bool                end = false;


    if (!SAVE_STATS_ONLY_)
    {
        f_cov_est_.open(sOUT_DIR_.c_str()+string("/cov_est.txt"));
        f_pf_stats_.open(sOUT_DIR_.c_str()+string("/PF_stats.txt"));
        f_odo_est_.open(sOUT_DIR_.c_str()+string("/odo_est.txt"));
    }


    size_t process_counter = step-rawlog_offset;
    while (!end)
    {
        // Finish if ESC is pushed:
        if (os::kbhit())
            if (os::getch()==27)
                end = true;

        // Load pose change from the rawlog:
        // ----------------------------------------
        CActionCollectionPtr action;
        CSensoryFramePtr     observations;
        CObservationPtr      obs;

        if (!CRawlog::getActionObservationPairOrObservation(
                    rawlog_in_stream,      // In stream
                    action, observations,  // Out pair <action,SF>, or:
                    obs,                   // Out single observation
                    rawlogEntry            // In/Out index counter.
                    ))
        {
            end = true;
            continue;
        }


        end = process(process_counter, action, observations, obs);
        process_counter++;

    }; // while rawlogEntries

    double repetitionTime = tictacGlobal_.Tac();

    // Avr. error:
    double covergenceErrorMean, covergenceErrorsMin,covergenceErrorsMax;
    math::confidenceIntervals(covergenceErrors_, covergenceErrorMean, covergenceErrorsMin,covergenceErrorsMax, STATS_CONF_INTERVAL_);

    // Save overall results:
    {

        CFileOutputStream f(format("%s_SUMMARY.txt",OUT_DIR_PREFIX.c_str()), true /* append */);

        f.printf("%% Ratio_covergence_success  #particles  time_for_execution  convergence_mean_error convergence_error_conf_int_inf convergence_error_conf_int_sup \n");
        if (!nConvergenceTests_) nConvergenceTests_=1;
        f.printf("%f %u %f %f %f %f\n",
                 ((double)nConvergenceOK_)/nConvergenceTests_,
                 PARTICLE_COUNT,
                 repetitionTime,
                 covergenceErrorMean,
                 covergenceErrorsMin,covergenceErrorsMax );
    }

    printf("\n TOTAL EXECUTION TIME = %.06f sec\n", repetitionTime );


    if (win3D_)
        mrpt::system::pause();
}
bool PFLocalization::process(size_t process_counter, CActionCollectionPtr action, CSensoryFramePtr observations, CObservationPtr obs){


    // Determine if we are reading a Act-SF or an Obs-only rawlog:
    if (obs)
    {
        // It's an observation-only rawlog: build an auxiliary pair of action-SF, since
        //  montecarlo-localization only accepts those pairs as input:

        // SF: Just one observation:
        // ------------------------------------------------------
        observations = CSensoryFrame::Create();
        observations->insert(obs);

        // ActionCollection: Just one action with a dummy odometry
        // ------------------------------------------------------
        action       = CActionCollection::Create();

        CActionRobotMovement2D dummy_odom;

        // TODO: Another good idea would be to take CObservationOdometry objects and use that information, if available.
        dummy_odom.computeFromOdometry(CPose2D(0,0,0),dummy_odom_params_);
        action->insert(dummy_odom);
    }
    else
    {
        // Already in Act-SF format, nothing else to do!
    }

    CPose2D     expectedPose; // Ground truth

    TTimeStamp cur_obs_timestamp;
    if (observations->size()>0)
        cur_obs_timestamp = observations->getObservationByIndex(0)->timestamp;

    if (process_counter >= 0)
    {
        // Do not execute the PF at "step=0", to let the initial PDF to be
        //   reflected in the logs.
        if (process_counter > 0)
        {
            // Show 3D?
            if (SHOW_PROGRESS_3D_REAL_TIME_)
            {
                CPose2D       meanPose;
                CMatrixDouble33 cov;
                pdf_.getCovarianceAndMean(cov,meanPose);

                COpenGLScenePtr ptrScene = win3D_->get3DSceneAndLock();

                win3D_->setCameraPointingToPoint(meanPose.x(),meanPose.y(),0);

                win3D_->addTextMessage(
                            10,10, mrpt::format("timestamp: %s", mrpt::system::dateTimeLocalToString(cur_obs_timestamp).c_str() ),
                            mrpt::utils::TColorf(.8f,.8f,.8f),
                            "mono", 15, mrpt::opengl::NICE, 6001 );

                win3D_->addTextMessage(
                            10,33, mrpt::format("#particles= %7u", static_cast<unsigned int>(pdf_.size()) ),
                            mrpt::utils::TColorf(.8f,.8f,.8f),
                            "mono", 15, mrpt::opengl::NICE, 6002 );

                win3D_->addTextMessage(
                            10,55, mrpt::format("mean pose (x y phi_deg)= %s", meanPose.asString().c_str() ),
                            mrpt::utils::TColorf(.8f,.8f,.8f),
                            "mono", 15, mrpt::opengl::NICE, 6003 );

                // The Ground Truth (GT):
                {
                    CRenderizablePtr GTpt = ptrScene->getByName("GT");
                    if (!GTpt)
                    {
                        GTpt = CDisk::Create();
                        GTpt->setName( "GT" );
                        GTpt->setColor(0,0,0, 0.9);

                        getAs<CDisk>(GTpt)->setDiskRadius(0.04);
                        ptrScene->insert( GTpt );
                    }

                    GTpt->setPose( expectedPose );
                }


                // The particles:
                {
                    CRenderizablePtr parts = ptrScene->getByName("particles");
                    if (parts) ptrScene->removeObject(parts);

                    CSetOfObjectsPtr p = pdf_.getAs3DObject<CSetOfObjectsPtr>();
                    p->setName("particles");
                    ptrScene->insert(p);
                }

                // The particles' cov:
                {
                    CRenderizablePtr    ellip = ptrScene->getByName("parts_cov");
                    if (!ellip)
                    {
                        ellip = CEllipsoid::Create();
                        ellip->setName( "parts_cov");
                        ellip->setColor(1,0,0, 0.6);

                        getAs<CEllipsoid>(ellip)->setLineWidth(2);
                        getAs<CEllipsoid>(ellip)->setQuantiles(3);
                        getAs<CEllipsoid>(ellip)->set2DsegmentsCount(60);
                        ptrScene->insert( ellip );
                    }
                    ellip->setLocation(meanPose.x(), meanPose.y(), 0.05 );

                    getAs<CEllipsoid>(ellip)->setCovMatrix(cov,2);
                }


                // The laser scan:
                {
                    CRenderizablePtr scanPts = ptrScene->getByName("scan");
                    if (!scanPts)
                    {
                        scanPts = CPointCloud::Create();
                        scanPts->setName( "scan" );
                        scanPts->setColor(1,0,0, 0.9);
                        getAs<CPointCloud>(scanPts)->enableColorFromZ(false);
                        getAs<CPointCloud>(scanPts)->setPointSize(4);
                        ptrScene->insert(scanPts);
                    }

                    CSimplePointsMap    map;
                    static CSimplePointsMap last_map;

                    CPose3D             robotPose3D( meanPose );

                    map.clear();
                    observations->insertObservationsInto( &map );

                    getAs<CPointCloud>(scanPts)->loadFromPointsMap( &last_map );
                    getAs<CPointCloud>(scanPts)->setPose( robotPose3D );
                    last_map = map;
                }

                // The camera:
                ptrScene->enableFollowCamera(true);

                // Views:
                COpenGLViewportPtr view1= ptrScene->getViewport("main");
                {
                    CCamera  &cam = view1->getCamera();
                    cam.setAzimuthDegrees(-90);
                    cam.setElevationDegrees(90);
                    cam.setPointingAt( meanPose );
                    cam.setZoomDistance(5);
                    cam.setOrthogonal();
                }

                /*COpenGLViewportPtr view2= ptrScene->createViewport("small_view"); // Create, or get existing one.
                            view2->setCloneView("main");
                            view2->setCloneCamera(false);
                            view2->setBorderSize(3);
                            {
                                CCamera  &cam = view1->getCamera();
                                cam.setAzimuthDegrees(-90);
                                cam.setElevationDegrees(90);
                                cam.setPointingAt( meanPose );
                                cam.setZoomDistance(15);
                                cam.setOrthogonal();

                                view2->setTransparent(false);
                                view2->setViewportPosition(0.59,0.01,0.4,0.3);
                            }*/

                win3D_->unlockAccess3DScene();

                // Move camera:
                //win3D_->setCameraPointingToPoint( curRobotPose.x, curRobotPose.y, curRobotPose.z );

                // Update:
                win3D_->forceRepaint();

                sleep( SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS_ );
            } // end show 3D real-time



            // ----------------------------------------
            // RUN ONE STEP OF THE PARTICLE FILTER:
            // ----------------------------------------
            tictac_.Tic();
            if (!SAVE_STATS_ONLY_)
                printf("Step %u -- Executing ParticleFilter on %u particles....",(unsigned int)process_counter, (unsigned int)pdf_.particlesCount());

            pf_.executeOn(
                        pdf_,
                        action.pointer(),           // Action
                        observations.pointer(), // Obs.
                        &pf_stats_       // Output statistics
                        );

            if (!SAVE_STATS_ONLY_)
                printf(" Done! in %.03fms, ESS=%f\n", 1000.0f*tictac_.Tac(), pdf_.ESS());
        }

        // Avrg. error:
        // ----------------------------------------
        CActionRobotMovement2DPtr best_mov_estim = action->getBestMovementEstimation();
        if (best_mov_estim)
            odometryEstimation_ = odometryEstimation_ + best_mov_estim->poseChange->getMeanVal();

        pdf_.getMean( pdfEstimation_ );

#if 1
        {   // Averaged error to GT
            double sumW=0;
            double locErr=0;
            for (size_t k=0; k<pdf_.size(); k++) sumW+=exp(pdf_.getW(k));
            for (size_t k=0; k<pdf_.size(); k++)
                locErr+= expectedPose.distanceTo( pdf_.getParticlePose(k) ) * exp(pdf_.getW(k))/ sumW;
            covergenceErrors_.push_back( locErr );
        }
#else
        // Error of the mean to GT
        covergenceErrors.push_back( expectedPose.distanceTo( pdfEstimation_ ) );
#endif

        // Text output:
        // ----------------------------------------
        if (!SAVE_STATS_ONLY_)
        {
            cout << "    Odometry est: " << odometryEstimation_ << "\n";
            cout << "         PDF est: " << pdfEstimation_ << ", ESS (B.R.)= " << pf_stats_.ESS_beforeResample << "\n";
            if (groundTruth_.getRowCount()>0)
                cout << "    Ground truth: " << expectedPose << "\n";
        }

        pdf_.getCovariance(covEstimation_);

        if (!SAVE_STATS_ONLY_)
        {
            f_cov_est_.printf("%e\n",sqrt(covEstimation_.det()) );
            f_pf_stats_.printf("%u %e %e\n",
                              (unsigned int)pdf_.size(),
                              pf_stats_.ESS_beforeResample,
                              pf_stats_.weightsVariance_beforeResample );
            f_odo_est_.printf("%f %f %f\n",odometryEstimation_.x(),odometryEstimation_.y(),odometryEstimation_.phi());
        }

        CPose2D meanPose;
        CMatrixDouble33 cov;
        pdf_.getCovarianceAndMean(cov,meanPose);

        if ( !SAVE_STATS_ONLY_ && SCENE3D_FREQ_>0 && (process_counter % SCENE3D_FREQ_)==0)
        {
            // Generate 3D scene:
            // ------------------------------
            MRPT_TODO("Someday I should clean up this mess, since two different 3D scenes are built -> refactor code")

                    // The Ground Truth (GT):
            {
                CRenderizablePtr GTpt = scene_.getByName("GT");
                if (!GTpt)
                {
                    GTpt = CDisk::Create();
                    GTpt = CDisk::Create();
                    GTpt->setName( "GT" );
                    GTpt->setColor(0,0,0, 0.9);

                    getAs<CDisk>(GTpt)->setDiskRadius(0.04);
                    scene_.insert( GTpt );
                }

                GTpt->setPose(expectedPose);
            }

            // The particles:
            {
                CRenderizablePtr parts = scene_.getByName("particles");
                if (parts) scene_.removeObject(parts);

                CSetOfObjectsPtr p = pdf_.getAs3DObject<CSetOfObjectsPtr>();
                p->setName("particles");
                scene_.insert(p);
            }

            // The particles' cov:
            {
                CRenderizablePtr    ellip = scene_.getByName("parts_cov");
                if (!ellip)
                {
                    ellip = CEllipsoid::Create();
                    ellip->setName( "parts_cov");
                    ellip->setColor(1,0,0, 0.6);

                    getAs<CEllipsoid>(ellip)->setLineWidth(4);
                    getAs<CEllipsoid>(ellip)->setQuantiles(3);
                    getAs<CEllipsoid>(ellip)->set2DsegmentsCount(60);
                    scene_.insert( ellip );
                }
                ellip->setLocation(meanPose.x(),meanPose.y(),0);

                getAs<CEllipsoid>(ellip)->setCovMatrix(cov,2);
            }


            // The laser scan:
            {
                CRenderizablePtr scanPts = scene_.getByName("scan");
                if (!scanPts)
                {
                    scanPts = CPointCloud::Create();
                    scanPts->setName( "scan" );
                    scanPts->setColor(1,0,0, 0.9);
                    getAs<CPointCloud>(scanPts)->enableColorFromZ(false);
                    getAs<CPointCloud>(scanPts)->setPointSize(4);
                    scene_.insert(scanPts);
                }

                CSimplePointsMap    map;
                static CSimplePointsMap last_map;

                CPose3D             robotPose3D( meanPose );

                map.clear();
                observations->insertObservationsInto( &map );

                getAs<CPointCloud>(scanPts)->loadFromPointsMap( &last_map );
                getAs<CPointCloud>(scanPts)->setPose( robotPose3D );
                last_map = map;
            }

            // The camera:
            scene_.enableFollowCamera(SCENE3D_FOLLOW_);

            // Views:
            COpenGLViewportPtr view1= scene_.getViewport("main");
            {
                CCamera  &cam = view1->getCamera();
                cam.setAzimuthDegrees(-90);
                cam.setElevationDegrees(90);
                cam.setPointingAt( meanPose);
                cam.setZoomDistance(5);
                cam.setOrthogonal();
            }

            /*COpenGLViewportPtr view2= scene_.createViewport("small_view"); // Create, or get existing one.
                        view2->setCloneView("main");
                        view2->setCloneCamera(false);
                        view2->setBorderSize(3);
                        {
                            CCamera  &cam = view1->getCamera();
                            cam.setAzimuthDegrees(-90);
                            cam.setElevationDegrees(90);
                            cam.setPointingAt( meanPose );
                            cam.setZoomDistance(15);
                            cam.setOrthogonal();

                            view2->setTransparent(false);
                            view2->setViewportPosition(0.59,0.01,0.4,0.3);
                        }*/
        }

        if (!SAVE_STATS_ONLY_ && SCENE3D_FREQ_ !=-1 && (process_counter % SCENE3D_FREQ_)==0)
        {
            // Save 3D scene:
            CFileGZOutputStream(format("%s/progress_%03u.3Dscene",sOUT_DIR_3D_.c_str(),(unsigned)process_counter)) << scene_;

            // Generate text files for matlab:
            // ------------------------------------
            pdf_.saveToTextFile(format("%s/particles_%03u.txt",sOUT_DIR_PARTS_.c_str(),(unsigned)process_counter));

            if (IS_CLASS(*observations->begin(),CObservation2DRangeScan))
            {
                CObservation2DRangeScanPtr o = CObservation2DRangeScanPtr( *observations->begin() );
                vectorToTextFile(o->scan , format("%s/observation_scan_%03u.txt",sOUT_DIR_PARTS_.c_str(),(unsigned)process_counter) );
            }
        }

    }
    // end if rawlog_offset
    // Test for end condition if we are testing convergence:
    if ( process_counter == testConvergenceAt_ )
    {
        nConvergenceTests_++;

        // Convergence??
        if ( sqrt(covEstimation_.det()) < 2 )
        {
            if ( pdfEstimation_.distanceTo(expectedPose) < 1.00f )
                nConvergenceOK_++;
        }
        return true;
    }
    return false;
}

