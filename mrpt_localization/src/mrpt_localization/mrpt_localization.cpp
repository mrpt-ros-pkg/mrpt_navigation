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

#include <mrpt/base.h>
#include <mrpt/slam.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace mrpt::random;
using namespace std;


PFLocalization::~PFLocalization()
{
}

PFLocalization::PFLocalization(Parameters *param)
    : PFLocalizationCore(), param_(param) {
}

void PFLocalization::incommingLaserData(mrpt::slam::CObservation2DRangeScanPtr _laser) {
    mrpt::slam::CSensoryFramePtr sf = mrpt::slam::CSensoryFrame::Create();
    mrpt::slam::CObservationPtr obs = mrpt::slam::CObservationPtr(_laser);
    sf->insert(obs);
    mrpt::poses::CPose2D incOdoPose;
    if(odomLastPoseLaser_.empty()) {
        odomLastPoseLaser_ = odomLastPoseMsg_.odometry;
    }
    incOdoPose = odomLastPoseMsg_.odometry - odomLastPoseLaser_;

    mrpt::slam::CActionRobotMovement2D odom_move;
    odom_move.timestamp = _laser->timestamp;
    odom_move.computeFromOdometry(incOdoPose, param_->motionModelOptions);
    mrpt::slam::CActionCollectionPtr action = mrpt::slam::CActionCollection::Create();
    action->insert(odom_move);
    process(action, sf, obs);
    odomLastPoseLaser_ = odomLastPoseLaser_;
    process_counter_++;
}

void PFLocalization::incommingOdomData(mrpt::slam::CObservationOdometryPtr _odometry) {

    odomLastPoseMsg_ = *_odometry;
}

void PFLocalization::init() {
    printf("iniFile ready %s\n", param_->iniFile.c_str());
    ASSERT_FILE_EXISTS_(param_->iniFile);
    printf("ASSERT_FILE_EXISTS_ %s\n", param_->iniFile.c_str());
    mrpt::utils::CConfigFile iniFile;
    iniFile.setFileName(param_->iniFile);
    printf("CConfigFile %s\n", param_->iniFile.c_str());


    vector_int          particles_count;    // Number of initial particles (if size>1, run the experiments N times)

    // Load configuration:
    // -----------------------------------------
    string iniSectionName ( "LocalizationExperiment" );
    process_counter_ = 0;

    // Mandatory entries:
    iniFile.read_vector(iniSectionName, "particles_count", vector_int(1,0), particles_count, /*Fail if not found*/true );
    OUT_DIR_PREFIX_      = iniFile.read_string(iniSectionName,"logOutput_dir","", /*Fail if not found*/true );


    if (param_->rawlogFile.empty()) {
        param_->rawlogFile = iniFile.read_string(iniSectionName,"rawlog_file","", /*Fail if not found*/true );
    }

    string      MAP_FILE;
    if (param_->mapFile.empty())
        MAP_FILE = iniFile.read_string(iniSectionName,"map_file","" );
    else MAP_FILE = param_->mapFile;

    // Non-mandatory entries:
    rawlog_offset_       = iniFile.read_int(iniSectionName,"rawlog_offset",0);
    string      GT_FILE             = iniFile.read_string(iniSectionName,"ground_truth_path_file","");
    SCENE3D_FREQ_        = iniFile.read_int(iniSectionName,"3DSceneFrequency",10);
    SCENE3D_FOLLOW_ = iniFile.read_bool(iniSectionName,"3DSceneFollowRobot",true);
    testConvergenceAt_   = iniFile.read_int(iniSectionName,"experimentTestConvergenceAtStep",-1);

    SAVE_STATS_ONLY_ = iniFile.read_bool(iniSectionName,"SAVE_STATS_ONLY",false);
    SHOW_PROGRESS_3D_REAL_TIME_ = iniFile.read_bool(iniSectionName,"SHOW_PROGRESS_3D_REAL_TIME",false);
    SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS_ = iniFile.read_int(iniSectionName,"SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS",1);
    STATS_CONF_INTERVAL_ = iniFile.read_double(iniSectionName,"STATS_CONF_INTERVAL",0.2);

#if !MRPT_HAS_WXWIDGETS
    SHOW_PROGRESS_3D_REAL_TIME = false;
#endif

    // Default odometry uncertainty parameters in "dummy_odom_params" depending on how fast the robot moves, etc...
    //  Only used for observations-only rawlogs:

    dummy_odom_params_.modelSelection = CActionRobotMovement2D::mmGaussian;
    dummy_odom_params_.gausianModel.minStdXY  = iniFile.read_double("DummyOdometryParams","minStdXY",0.04);
    dummy_odom_params_.gausianModel.minStdPHI = DEG2RAD(iniFile.read_double("DummyOdometryParams","minStdPHI", 2.0));





    cout<< "-------------------------------------------------------------\n"
        << "\t RAWLOG_FILE = \t "   << param_->rawlogFile << endl
        << "\t MAP_FILE = \t "      << MAP_FILE << endl
        << "\t GT_FILE = \t "       << GT_FILE << endl
        << "\t OUT_DIR_PREFIX = \t "<< OUT_DIR_PREFIX_ << endl
        << "\t #particles = \t "    << particles_count << endl
        << "-------------------------------------------------------------\n";


    // --------------------------------------------------------------------
    //                      EXPERIMENT PREPARATION
    // --------------------------------------------------------------------

    configureFilter(iniFile);
    // Metric map options:
    loadMap(MAP_FILE, iniFile);

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




    INITIAL_PARTICLE_COUNT_ = *particles_count.begin();

    //show3DDebug();



    // Global stats for all the experiment loops:
    nConvergenceTests_ = 0;
    nConvergenceOK_ = 0;
    covergenceErrors_.clear();

    // --------------------------------------------------------------------
    //                  EXPERIMENT REPETITIONS LOOP
    // --------------------------------------------------------------------

    // The experiment directory is:

    initializeFilter(iniFile, iniSectionName);
    initLog();
    init3DDebug();

    if(param_->rawlogFile.empty()) {
        // There was no rawlog therfore I am waiting for ros massages
    } else {
        // There was no rawlog therfore I am waiting for ros massages
        playRawlog();
    }
}

bool PFLocalization::playRawlog() {
    // --------------------------
    // Load the rawlog:
    // --------------------------
    printf("Opening the rawlog file...");
    CFileGZInputStream rawlog_in_stream(param_->rawlogFile);
    printf("OK\n");

    bool                end = false;
    size_t  step = 0;
    size_t rawlogEntry = 0;
    process_counter_ = step-rawlog_offset_;
    mrpt::utils::CTicTac tictacRawLog;
    tictacRawLog.Tic();
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


        end = process(action, observations, obs);
        process_counter_++;

    }; // while rawlogEntries
    double repetitionTime = tictacRawLog.Tac();

    // Avr. error:
    double covergenceErrorMean, covergenceErrorsMin,covergenceErrorsMax;
    math::confidenceIntervals(covergenceErrors_, covergenceErrorMean, covergenceErrorsMin,covergenceErrorsMax, STATS_CONF_INTERVAL_);

    // Save overall results:
    {

        CFileOutputStream f(format("%s_SUMMARY.txt",OUT_DIR_PREFIX_.c_str()), true /* append */);

        f.printf("%% Ratio_covergence_success  #particles  time_for_execution  convergence_mean_error convergence_error_conf_int_inf convergence_error_conf_int_sup \n");
        if (!nConvergenceTests_) nConvergenceTests_=1;
        f.printf("%f %u %f %f %f %f\n",
                 ((double)nConvergenceOK_)/nConvergenceTests_,
                 INITIAL_PARTICLE_COUNT_,
                 repetitionTime,
                 covergenceErrorMean,
                 covergenceErrorsMin,covergenceErrorsMax );
    }

    printf("\n TOTAL EXECUTION TIME = %.06f sec\n", repetitionTime );

    if (win3D_)
        mrpt::system::pause();

}


bool PFLocalization::process(CActionCollectionPtr _action, CSensoryFramePtr _observations, CObservationPtr _obs) {


    // Determine if we are reading a Act-SF or an Obs-only rawlog:
    if (_obs)
    {
        // It's an observation-only rawlog: build an auxiliary pair of action-SF, since
        //  montecarlo-localization only accepts those pairs as input:

        // SF: Just one observation:
        // ------------------------------------------------------
        _observations = CSensoryFrame::Create();
        _observations->insert(_obs);

        // ActionCollection: Just one action with a dummy odometry
        // ------------------------------------------------------
        _action       = CActionCollection::Create();

        CActionRobotMovement2D dummy_odom;

        // TODO: Another good idea would be to take CObservationOdometry objects and use that information, if available.
        dummy_odom.computeFromOdometry(CPose2D(0,0,0),dummy_odom_params_);
        _action->insert(dummy_odom);
    }
    else
    {
        // Already in Act-SF format, nothing else to do!
    }

    if (process_counter_ >= 0)
    {
        // Do not execute the PF at "step=0", to let the initial PDF to be
        //   reflected in the logs.
        if (process_counter_ > 0)
        {
            show3DDebugPreprocess(_observations);
            // ----------------------------------------
            // RUN ONE STEP OF THE PARTICLE FILTER:
            // ----------------------------------------
            tictac_.Tic();
            if (!SAVE_STATS_ONLY_)
                printf("Step %u -- Executing ParticleFilter on %u particles....",(unsigned int)process_counter_, (unsigned int)pdf_.particlesCount());

            pf_.executeOn(
                pdf_,
                _action.pointer(),           // Action
                _observations.pointer(), // Obs.
                &pf_stats_       // Output statistics
            );

            if (!SAVE_STATS_ONLY_)
                printf(" Done! in %.03fms, ESS=%f\n", 1000.0f*tictac_.Tac(), pdf_.ESS());
        }


        // Avrg. error:
        // ----------------------------------------
        CActionRobotMovement2DPtr best_mov_estim = _action->getBestMovementEstimation();
        if (best_mov_estim)
            odometryEstimation_ = odometryEstimation_ + best_mov_estim->poseChange->getMeanVal();

        pdf_.getMean( pdfEstimation_ );
        pdf_.getCovariance(covEstimation_);

#if 1
        {   // Averaged error to GT
            double sumW=0;
            double locErr=0;
            for (size_t k=0; k<pdf_.size(); k++) sumW+=exp(pdf_.getW(k));
            for (size_t k=0; k<pdf_.size(); k++)
                locErr+= expectedPose_.distanceTo( pdf_.getParticlePose(k) ) * exp(pdf_.getW(k))/ sumW;
            covergenceErrors_.push_back( locErr );
        }
#else
        // Error of the mean to GT
        covergenceErrors.push_back( expectedPose_.distanceTo( pdfEstimation_ ) );
#endif

        show3DDebugPostprocess(_observations);
        logResults(_observations);

    }
    // Test for end condition if we are testing convergence:
    if ( process_counter_ == testConvergenceAt_ )
    {
        nConvergenceTests_++;

        // Convergence??
        if ( sqrt(covEstimation_.det()) < 2 )
        {
            if ( pdfEstimation_.distanceTo(expectedPose_) < 1.00f )
                nConvergenceOK_++;
        }
        return true;
    }
    return false;
}

void PFLocalization::logResults(CSensoryFramePtr _observations) {
        // Text output:
        // ----------------------------------------
        if (!SAVE_STATS_ONLY_)
        {
            cout << "    Odometry est: " << odometryEstimation_ << "\n";
            cout << "         PDF est: " << pdfEstimation_ << ", ESS (B.R.)= " << pf_stats_.ESS_beforeResample << "\n";
            if (groundTruth_.getRowCount()>0)
                cout << "    Ground truth: " << expectedPose_ << "\n";
        }


        if (!SAVE_STATS_ONLY_)
        {
            f_cov_est_.printf("%e\n",sqrt(covEstimation_.det()) );
            f_pf_stats_.printf("%u %e %e\n",
                               (unsigned int)pdf_.size(),
                               pf_stats_.ESS_beforeResample,
                               pf_stats_.weightsVariance_beforeResample );
            f_odo_est_.printf("%f %f %f\n",odometryEstimation_.x(),odometryEstimation_.y(),odometryEstimation_.phi());
        }


        if (!SAVE_STATS_ONLY_ && SCENE3D_FREQ_ !=-1 && (process_counter_ % SCENE3D_FREQ_)==0)
        {
            // Save 3D scene:
            CFileGZOutputStream(format("%s/progress_%03u.3Dscene",sOUT_DIR_3D_.c_str(),(unsigned)process_counter_)) << scene_;

            // Generate text files for matlab:
            // ------------------------------------
            pdf_.saveToTextFile(format("%s/particles_%03u.txt",sOUT_DIR_PARTS_.c_str(),(unsigned)process_counter_));

            if (IS_CLASS(*_observations->begin(),CObservation2DRangeScan))
            {
                CObservation2DRangeScanPtr o = CObservation2DRangeScanPtr( *_observations->begin() );
                vectorToTextFile(o->scan , format("%s/observation_scan_%03u.txt",sOUT_DIR_PARTS_.c_str(),(unsigned)process_counter_) );
            }
        }
}

void PFLocalization::loadMap(const std::string &_mapFilename, const mrpt::utils::CConfigFile &_configFile) {

    TSetOfMetricMapInitializers mapInitializers;
    mapInitializers.loadFromConfigFile( _configFile,"metricMap");
    if(param_->debug) mapInitializers.dumpToConsole();

    CSimpleMap  simpleMap;

    // Load the set of metric maps to consider in the experiments:
    metricMap_.setListOfMaps( &mapInitializers );
    if(param_->debug) mapInitializers.dumpToConsole();

    randomGenerator.randomize();

    // Load the map (if any):
    // -------------------------
    if (_mapFilename.size())
    {
        ASSERT_( fileExists(_mapFilename) );

        // Detect file extension:
        // -----------------------------
        string mapExt = lowerCase( extractFileExtension( _mapFilename, true ) ); // Ignore possible .gz extensions

        if ( !mapExt.compare( "simplemap" ) )
        {
            // It's a ".simplemap":
            // -------------------------
            if(param_->debug) ("Loading '.simplemap' file...");
            CFileGZInputStream(_mapFilename) >> simpleMap;
            printf("Ok\n");

            ASSERT_( simpleMap.size()>0 );

            // Build metric map:
            // ------------------------------
            if(param_->debug) printf("Building metric map(s) from '.simplemap'...");
            metricMap_.loadFromProbabilisticPosesAndObservations(simpleMap);
            if(param_->debug) printf("Ok\n");
        }
        else if ( !mapExt.compare( "gridmap" ) )
        {
            // It's a ".gridmap":
            // -------------------------
            if(param_->debug) printf("Loading gridmap from '.gridmap'...");
            ASSERT_( metricMap_.m_gridMaps.size()==1 );
            CFileGZInputStream(_mapFilename) >> (*metricMap_.m_gridMaps[0]);
            if(param_->debug) printf("Ok\n");
        }
        else
        {
            THROW_EXCEPTION_CUSTOM_MSG1("Map file has unknown extension: '%s'",mapExt.c_str());
        }

    }
}

void PFLocalization::configureFilter(const mrpt::utils::CConfigFile &_configFile) {

    // PF-algorithm Options:
    // ---------------------------
    CParticleFilter::TParticleFilterOptions     pfOptions;
    pfOptions.loadFromConfigFile( _configFile, "PF_options" );
    pfOptions.dumpToConsole();

    // PDF Options:
    // ------------------
    TMonteCarloLocalizationParams   pdfPredictionOptions;
    pdfPredictionOptions.KLD_params.loadFromConfigFile( _configFile, "KLD_options");

    MRPT_TODO("Max to Jose: can I reinizialize the pdf like this, should we add a function or should I use a ptr?");
    pdf_.clear();

    // PDF Options:
    pdf_.options = pdfPredictionOptions;

    pdf_.options.metricMap = &metricMap_;

    // Create the PF object:
    pf_.m_options = pfOptions;
}

void PFLocalization::initLog() {
    if (SAVE_STATS_ONLY_) return;

    sOUT_DIR_        = format("%s",OUT_DIR_PREFIX_.c_str());
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

    f_cov_est_.open(sOUT_DIR_.c_str()+string("/cov_est.txt"));
    f_pf_stats_.open(sOUT_DIR_.c_str()+string("/PF_stats.txt"));
    f_odo_est_.open(sOUT_DIR_.c_str()+string("/odo_est.txt"));
}

void PFLocalization::initializeFilter(const mrpt::utils::CConfigFile &_configFile, const std::string &_sectionName) {

// Initialize the PDF:
// -----------------------------
    tictac_.Tic();
    if ( !_configFile.read_bool(_sectionName,"init_PDF_mode",false, /*Fail if not found*/true) )
        pdf_.resetUniformFreeSpace(
            metricMap_.m_gridMaps[0].pointer(),
            0.7f,
            INITIAL_PARTICLE_COUNT_ ,
            _configFile.read_float(_sectionName,"init_PDF_min_x",0,true),
            _configFile.read_float(_sectionName,"init_PDF_max_x",0,true),
            _configFile.read_float(_sectionName,"init_PDF_min_y",0,true),
            _configFile.read_float(_sectionName,"init_PDF_max_y",0,true),
            DEG2RAD(_configFile.read_float(_sectionName,"init_PDF_min_phi_deg",-180)),
            DEG2RAD(_configFile.read_float(_sectionName,"init_PDF_max_phi_deg",180))
        );
    else
        pdf_.resetUniform(
            _configFile.read_float(_sectionName,"init_PDF_min_x",0,true),
            _configFile.read_float(_sectionName,"init_PDF_max_x",0,true),
            _configFile.read_float(_sectionName,"init_PDF_min_y",0,true),
            _configFile.read_float(_sectionName,"init_PDF_max_y",0,true),
            DEG2RAD(_configFile.read_float(_sectionName,"init_PDF_min_phi_deg",-180)),
            DEG2RAD(_configFile.read_float(_sectionName,"init_PDF_max_phi_deg",180)),
            INITIAL_PARTICLE_COUNT_
        );
    printf("PDF of %u particles initialized in %.03fms\n", INITIAL_PARTICLE_COUNT_, 1000*tictac_.Tac());
}

void PFLocalization::init3DDebug() {
    if (!SHOW_PROGRESS_3D_REAL_TIME_) return;
    if(!win3D_) {
        win3D_ = CDisplayWindow3D::Create("pf-localization - The MRPT project", 1000, 600);
        win3D_->setCameraZoom(20);
        win3D_->setCameraAzimuthDeg(-45);
        //win3D_->waitForKey();

        // Create the 3D scene and get the map only once, later we'll modify only the particles, etc..
        mrpt::slam::COccupancyGridMap2D::TEntropyInfo gridInfo;
        // The gridmap:
        if (metricMap_.m_gridMaps.size())
        {
            metricMap_.m_gridMaps[0]->computeEntropy( gridInfo );
            printf("The gridmap has %.04fm2 observed area, %u observed cells\n", gridInfo.effectiveMappedArea, (unsigned) gridInfo.effectiveMappedCells );

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
        printf("Initial PDF: %f particles/m2\n", INITIAL_PARTICLE_COUNT_/gridInfo.effectiveMappedArea);
    } // Show 3D?
}

void PFLocalization::show3DDebugPreprocess(CSensoryFramePtr _observations) {
    // Create 3D window if requested:
    if (SHOW_PROGRESS_3D_REAL_TIME_)
    {
        TTimeStamp cur_obs_timestamp;
        if (_observations->size()>0)
            cur_obs_timestamp = _observations->getObservationByIndex(0)->timestamp;

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
            GTpt->setPose( expectedPose_ );
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
            _observations->insertObservationsInto( &map );

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
    }
}
void PFLocalization::show3DDebugPostprocess(CSensoryFramePtr _observations) {

    CPose2D       meanPose;
    CMatrixDouble33 cov;
    pdf_.getCovarianceAndMean(cov,meanPose);

    if ( !SAVE_STATS_ONLY_ && SCENE3D_FREQ_>0 && (process_counter_ % SCENE3D_FREQ_)==0)
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

            GTpt->setPose(expectedPose_);
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
            _observations->insertObservationsInto( &map );

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

} // end show 3D real-time

