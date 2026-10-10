/**
* Description: Pure SLAM data processing without ROS2 dependencies
* This file handles point cloud processing and pose computation
*/

#include "SlamDataProcess.h"
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <chrono>
#include <memory>
#include <set>
#include "MapPoint.h"

using namespace std;

namespace ORB_SLAM3
{

SlamDataProcess::SlamDataProcess(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, 
                Tracking *pTracking, const string &strSettingPath, Atlas* pAtlas) :
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpTracker(pTracking), mpAtlas(pAtlas),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false),
    mbGetNewCamPose(false), mbGetPointCloud(false), mbGetDrawFrame(false), mbGetTrajectory(false)
{
    // Read settings from file
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    
    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }
    
    // Initialize camera-to-ground transformation
    // mInitCam2Ground_R << 1,0,0,0,0,1,0,-1,0;
    mInitCam2Ground_R << 0,0,1,-1,0,0,0,-1,0;
    mInitCam2Ground_t << 0, 0, 0;
    mTrans_cam2ground = Eigen::Matrix4f::Identity();
    mTrans_cam2ground.block<3,3>(0,0) = mInitCam2Ground_R;
    mTrans_cam2ground.block<3,1>(0,3) = mInitCam2Ground_t;

    // Initialize camera-to-vehicle transformation
    mCam2Vehicle_R << 0,0,1,-1,0,0,0,-1,0;
    mCam2Vehicle_t << 0.1, 0, 0.065;
    mTrans_cam2vehicle = Eigen::Matrix4f::Identity();
    mTrans_cam2vehicle.block<3,3>(0,0) = mCam2Vehicle_R;
    mTrans_cam2vehicle.block<3,1>(0,3) = mCam2Vehicle_t;
    
    // Initialize current pose matrices
    mCam2GroundNow_T = Eigen::Matrix4f::Identity();
    mVehicle2GroundNow_T = Eigen::Matrix4f::Identity();
    
    // Initialize point cloud mapping
    mpPointClouds = new PointClouds();
    
    // Initialize data structures
    mCurrentPoseData.has_new_pose = false;
    mCurrentPoseData.cam_pose_to_ground = Eigen::Matrix4f::Identity();
    mCurrentPoseData.vehicle_pose_to_ground = Eigen::Matrix4f::Identity();
    
    mCurrentPointCloudData.has_new_cloud = false;
    mCurrentPointCloudMappingData.has_new_map = false;
    mCurrentFrameData.has_new_frame = false;
    mCurrentTrajectoryData.has_new_trajectory = false;
    
    cout << "SlamDataProcess initialized for pure data processing (no ROS2)" << endl;
    cout << "Settings file: " << strSettingPath << endl;
}

SlamDataProcess::~SlamDataProcess()
{
    RequestFinish();
    
    if (mpPointClouds)
        delete mpPointClouds;
}

PointClouds* SlamDataProcess::getPointClouds()
{
    return mpPointClouds;
}

void SlamDataProcess::Run()
{
    mbFinished = false;
    mbStopped = false;
    
    cout << "SlamDataProcess: Starting data processing loop..." << endl;

    while (!CheckFinish())
    {
        {
            // Track/reset modifies trajectory lists and can delete atlas objects.
            std::unique_lock<std::mutex> tracking_lock(mpSystem->GetTrackingMutex());
            if (CheckFinish()) break;
            UpdateCurrentPose();
            UpdatePointCloud();
            UpdatePointCloudMapping();
            UpdateCurrentFrame();
            UpdateTrajectory();
            mbGetNewCamPose = false;
        }
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(mT));
    }

    cout << "SlamDataProcess: Processing loop finished" << endl;
    SetFinish();
}

void SlamDataProcess::UpdateCurrentPose()
{
    unique_lock<mutex> lock(mMutexCamera);
    if(!mCameraPose.empty() && mbGetNewCamPose)
    {
        Eigen::Matrix4f cam_pose2firstcam;
        {
            cv2eigen(mCameraPose.inv(),cam_pose2firstcam);
            mCurrentPoseData.cam_pose_to_ground = mTrans_cam2ground * cam_pose2firstcam;
            mCam2GroundNow_T.block<4,4>(0,0) = mCurrentPoseData.cam_pose_to_ground;

            mCurrentPoseData.vehicle_pose_to_ground = mCam2GroundNow_T * mTrans_cam2vehicle.inverse();
            mVehicle2GroundNow_T.block<4,4>(0,0) = mCurrentPoseData.vehicle_pose_to_ground;
        }
        
        mCurrentPoseData.has_new_pose = true;
        mbGetNewCamPose = true;
    }
}

void SlamDataProcess::UpdatePointCloudMapping()
{
    if (mpPointClouds && mbGetNewCamPose)
    {
        unique_lock<mutex> lock(mMutexPointCloudMapping);
        
        // Publish immutable snapshots: readers may still hold the previous cloud.
        mCurrentPointCloudMappingData.map_points = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

        if (mpAtlas)
        {
            // Generate colored point cloud from PointCloudMapping (similar to CloudPointMappingPub)
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            mpPointClouds->generatePointCloud(globalCloud, mpAtlas, mTrans_cam2ground);

            if (globalCloud && globalCloud->points.size() > 0)
            {
                // Copy to map_points
                *mCurrentPointCloudMappingData.map_points = *globalCloud;
            }
        }

        // Set timestamps and flags
        mCurrentPointCloudMappingData.timestamp = mCurrentPoseData.timestamp;
        mCurrentPointCloudMappingData.has_new_map = true;
    }
}

void SlamDataProcess::UpdatePointCloud()
{
    if(mpPointClouds && mbGetNewCamPose)
    {
        unique_lock<mutex> lock(mMutexPointCloud);
        
        // Initialize point clouds if not already done
        mCurrentPointCloudData.all_points = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        mCurrentPointCloudData.ref_points = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

        // Clear previous data
        mCurrentPointCloudData.all_points->clear();
        mCurrentPointCloudData.ref_points->clear();
        
        // Generate map points similar to GetCurrentROSAllPointCloud
        if (mpAtlas)
        {
            const vector<MapPoint*> &vpMPs = mpAtlas->GetAllMapPoints();
            const vector<MapPoint*> &vpRefMPs = mpAtlas->GetReferenceMapPoints();
            
            set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());
            
            if (!vpMPs.empty())
            {
                // Process all map points (excluding reference points for all_points)
                for (size_t i = 0; i < vpMPs.size(); i++)
                {
                    MapPoint* pMP = vpMPs[i];
                    if (pMP && !pMP->isBad() && spRefMPs.count(pMP) == 0)
                    {
                        Eigen::Vector3f pos = pMP->GetWorldPos();
                        
                        // Transform to ground frame
                        Eigen::Vector4f pos_homogeneous(pos.x(), pos.y(), pos.z(), 1.0f);
                        Eigen::Vector4f pos_ground = mTrans_cam2ground * pos_homogeneous;
                        
                        pcl::PointXYZRGB point;
                        point.x = pos_ground.x();
                        point.y = pos_ground.y();
                        point.z = pos_ground.z();
                        point.r = 255;  // Set red channel
                        point.g = 255;    // Set green channel
                        point.b = 255;    // Set blue channel
                        point.a = 255;  // Set alpha channel

                        mCurrentPointCloudData.all_points->push_back(point);
                    }
                }
                
                // Process reference map points
                for (const auto& refMP : spRefMPs)
                {
                    if (refMP && !refMP->isBad())
                    {
                        Eigen::Vector3f pos = refMP->GetWorldPos();
                        
                        // Transform to ground frame
                        Eigen::Vector4f pos_homogeneous(pos.x(), pos.y(), pos.z(), 1.0f);
                        Eigen::Vector4f pos_ground = mTrans_cam2ground * pos_homogeneous;
                        
                        pcl::PointXYZRGB point;
                        point.x = pos_ground.x();
                        point.y = pos_ground.y();
                        point.z = pos_ground.z();
                        point.r = 255;  // Set red channel
                        point.g = 0;    // Set green channel
                        point.b = 0;    // Set blue channel
                        point.a = 255;  // Set alpha channel

                        mCurrentPointCloudData.ref_points->push_back(point);
                    }
                }
            }
        }
        
        // Set timestamps and flags
        mCurrentPointCloudData.timestamp = mCurrentPoseData.timestamp;
        mCurrentPointCloudData.has_new_cloud = true;
    }
}

void SlamDataProcess::UpdateCurrentFrame()
{
    if(mpFrameDrawer)
    {
        unique_lock<mutex> lock(mMutexFrame);
        cv::Mat frame = mpFrameDrawer->DrawFrame();
        
        if(!frame.empty())
        {
            mCurrentFrameData.current_frame = frame.clone();
            mCurrentFrameData.has_new_frame = true;
            mCurrentFrameData.timestamp = mCurrentPoseData.timestamp;
        }
    }
}

void SlamDataProcess::UpdateTrajectory()
{
    if(!mCameraPose.empty() && mbGetNewCamPose)
    {
        unique_lock<mutex> lock(mMutexTrajectory);

        vector<cv::Mat> currentTrajectory;
        vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
        sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

        if (vpKFs.empty() || mpTracker->mlRelativeFramePoses.empty()) return;
        mCurrentTrajectoryData.camera_trajectory.clear();
        mCurrentTrajectoryData.vehicle_trajectory.clear();
        // All poses and point clouds use the same atlas frame.
        Sophus::SE3f Two;
        list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
        auto lost = mpTracker->mlbLost.begin();

        for(list<Sophus::SE3f>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend && lRit!=mpTracker->mlpReferences.end() && lost!=mpTracker->mlbLost.end();lit++, lRit++, lost++)
        {
            ORB_SLAM3::KeyFrame* pKF = *lRit;
            if (*lost || !pKF) continue;

            //cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);
            Sophus::SE3f Trw;

            while(pKF && pKF->isBad())
            {
            //  cout << "bad parent" << endl;
                Trw = Trw*pKF->mTcp;
                pKF = pKF->GetParent();
            }

            if (!pKF) continue;
            Trw = Trw*pKF->GetPose()*Two;  // keep the first frame on the origin
            //Trw = Trw*pKF->GetPose();

            Sophus::SE3f Tcw = (*lit)*Trw;
            //cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
            //cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

            currentTrajectory.push_back(Converter::toCvMat(Tcw.inverse().matrix()));
        }


        Eigen::Matrix4f cam_pose_temp;
        
        for(auto mt:currentTrajectory) // no need to inverse
        {
            cv::cv2eigen(mt,cam_pose_temp);
            Eigen::Matrix4f cam_pose2ground = mTrans_cam2ground * cam_pose_temp;
            Eigen::Matrix4f vehicle_pose2ground = cam_pose2ground * mTrans_cam2vehicle.inverse();

            mCurrentTrajectoryData.camera_trajectory.push_back(cam_pose2ground);
            mCurrentTrajectoryData.vehicle_trajectory.push_back(vehicle_pose2ground);
        }

        mCurrentTrajectoryData.has_new_trajectory = true;
        mCurrentTrajectoryData.timestamp = mCurrentPoseData.timestamp;
    }
}


void SlamDataProcess::SetCurrentCameraPose(const cv::Mat& Tcw, double timestamp)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
    mCurrentPoseData.timestamp = timestamp;
    mbGetNewCamPose = true;
}

// Public interface methods
SlamDataProcess::PoseData SlamDataProcess::GetCurrentPoseData()
{
    unique_lock<mutex> lock(mMutexCamera);
    PoseData data = mCurrentPoseData;
    mCurrentPoseData.has_new_pose = false; // Reset flag after reading
    return data;
}

SlamDataProcess::PointCloudData SlamDataProcess::GetCurrentPointCloudData()
{
    unique_lock<mutex> lock(mMutexPointCloud);
    PointCloudData data = mCurrentPointCloudData;
    mCurrentPointCloudData.has_new_cloud = false; // Reset flag after reading
    return data;
}

SlamDataProcess::PointCloudMappingData SlamDataProcess::GetCurrentPointCloudMappingData()
{
    unique_lock<mutex> lock(mMutexPointCloudMapping);
    PointCloudMappingData data = mCurrentPointCloudMappingData;
    mCurrentPointCloudMappingData.has_new_map = false; // Reset flag after reading
    return data;
}

SlamDataProcess::FrameData SlamDataProcess::GetCurrentFrameData()
{
    unique_lock<mutex> lock(mMutexFrame);
    FrameData data = mCurrentFrameData;
    mCurrentFrameData.has_new_frame = false; // Reset flag after reading
    return data;
}

SlamDataProcess::TrajectoryData SlamDataProcess::GetCurrentTrajectoryData()
{
    unique_lock<mutex> lock(mMutexTrajectory);
    TrajectoryData data = mCurrentTrajectoryData;
    mCurrentTrajectoryData.has_new_trajectory = false; // Reset flag after reading
    return data;
}

// Thread control methods
void SlamDataProcess::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool SlamDataProcess::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void SlamDataProcess::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool SlamDataProcess::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void SlamDataProcess::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
}

bool SlamDataProcess::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested)
    {
        mbStopped = true;
        return true;
    }
    return false;
}

bool SlamDataProcess::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

void SlamDataProcess::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
    mbStopRequested = false;
}

Eigen::Matrix4f SlamDataProcess::getTransCam2Ground() { return mTrans_cam2ground; }
Eigen::Matrix4f SlamDataProcess::getTransCam2Vehicle() { return mTrans_cam2vehicle; }

} // namespace ORB_SLAM3
