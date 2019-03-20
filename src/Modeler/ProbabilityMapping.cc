/*
 * =====================================================================================
 *
 *       Filename:  ProbabilityMapping.cc
 *
 *    Description:
 *
 *        Version:  0.1
 *        Created:  01/21/2016 10:39:12 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Josh Tang, Rebecca Frederick
 *
 *        version: 1.0
 *        created: 8/9/2016
 *        Log: fix a lot of bug, Almost rewrite the code.
 *
 *        author: He Yijia
 * *
 *        Version: 1.1
 *        Created: 05/18/2017
 *        Author: Shida He
 *
 * =====================================================================================
 */

#include <cmath>
#include <opencv2/opencv.hpp>
#include <numeric>
#include "Modeler/ProbabilityMapping.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "ORBmatcher.h"
#include "LocalMapping.h"
#include <stdint.h>
#include <stdio.h>


template<typename T>
float bilinear(const cv::Mat& img, const float& y, const float& x)
{
    int x0 = (int)std::floor(x);
    int y0 = (int )std::floor(y);
    int x1 = x0 + 1;
    int y1 =  y0 + 1;

    float x0_weight = x1 - x;
    float y0_weight = y1 - y;
    float x1_weight = 1.0f - x0_weight;
    float y1_weight = 1.0f - y0_weight;

    float interpolated = img.at<T>(y0,x0) * x0_weight * y0_weight +
                         img.at<T>(y0,x1) * x1_weight * y0_weight +
                         img.at<T>(y1,x0) * x0_weight * y1_weight +
                         img.at<T>(y1,x1) * x1_weight * y1_weight;

    return interpolated;
}

ProbabilityMapping::ProbabilityMapping(ORB_SLAM2::Map* pMap):mpMap(pMap)
{
}

void ProbabilityMapping::Run()
{
    while(1)
    {
        if(CheckFinish()) break;
        //TestSemiDenseViewer();
#ifdef OnlineLoop
//        struct timespec start, finish;
//        double duration;
//        clock_gettime(CLOCK_MONOTONIC, &start);

        SemiDenseLoop();

        //make point position dependent to kf position
        UpdateAllSemiDensePointSet();

//        clock_gettime(CLOCK_MONOTONIC, &finish);
//        duration = ( finish.tv_sec - start.tv_sec );
//        duration += ( finish.tv_nsec - start.tv_nsec ) / 1000000000.0;
//        std::cout<<"semi dense mapping took: "<< duration << "s" << std::endl;
#endif
        usleep(5000);
    }

#ifndef OnlineLoop


    SemiDenseLoop();

    clock_gettime(CLOCK_MONOTONIC, &finish);
    duration = ( finish.tv_sec - start.tv_sec );
    duration += ( finish.tv_nsec - start.tv_nsec ) / 1000000000.0;
    std::cout<<"semi dense mapping took: "<< duration << "s" << std::endl;
#endif

    std::string strFileName("semi_pointcloud.obj");
    std::ofstream fileOut(strFileName.c_str(), std::ios::out);
    if(!fileOut){
        std::cerr << "Failed to save points on line" << std::endl;
        return;
    }
    vector<ORB_SLAM2::KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    for (size_t indKF = 0; indKF < vpKFs.size(); indKF++) {
        ORB_SLAM2::KeyFrame* kf = vpKFs[indKF];
        if( kf->isBad() ) continue;
        if(! kf->semidense_flag_) continue;
        if(! kf->interKF_depth_flag_) continue;

        for(size_t y = 0; y< (size_t)kf->im_.rows; y++) {
            for (size_t x = 0; x < (size_t) kf->im_.cols; x++) {

                Eigen::Vector3f Pw(kf->SemiDensePointSets_.at<float>(y, 3 * x),
                                   kf->SemiDensePointSets_.at<float>(y, 3 * x + 1),
                                   kf->SemiDensePointSets_.at<float>(y, 3 * x + 2));
                if (kf->depth_sigma_.at<float>(y,x) > 0.01) continue;
                if (kf->depth_map_.at<float>(y,x) > 0.000001) {
                    fileOut << "v " + std::to_string(Pw[0]) + " " + std::to_string(Pw[1]) + " " + std::to_string(Pw[2])
                            << std::endl;
                }
            }
        }

    }

    fileOut.flush();
    fileOut.close();
    std::cout << "saved semi dense point cloud" << std::endl;

    mbFinished = true;
}

void ProbabilityMapping::SemiDenseLoop(ORB_SLAM2::KeyFrame* pKF){

    {
        ORB_SLAM2::KeyFrame *kf = pKF;
        if (kf->isBad() || kf->semidense_flag_)
            return;
//        // 10 keyframe delay
//        if (!kf->MappingIdDelay())
//            continue;

//        std::vector<ORB_SLAM2::KeyFrame*> closestMatches = kf->GetBestCovisibilityKeyFrames(covisN);
//        if(closestMatches.size() < covisN) {continue;}

        // use covisN good neighbor kfs
        std::vector<ORB_SLAM2::KeyFrame *> closestMatchesAll = kf->GetVectorCovisibleKeyFrames();
        std::vector<ORB_SLAM2::KeyFrame *> closestMatches;
        for (size_t idxCov = 0; idxCov < closestMatchesAll.size(); idxCov++) {
            if (closestMatches.size() >= covisN)
                break;
            if (closestMatchesAll[idxCov]->isBad()) continue;
            if (!closestMatchesAll[idxCov]->Mapped()) continue;
            closestMatches.push_back(closestMatchesAll[idxCov]);
        }
        if (closestMatches.size() < covisN) { continue; }

        // start timing

        struct timespec start, finish;
        double duration;
        clock_gettime(CLOCK_MONOTONIC, &start);

        cout << "semidense_Info:    vpKFs.size()--> " << vpKFs.size() << std::endl;

        std::map<ORB_SLAM2::KeyFrame *, float> rotIs;
        for (size_t j = 0; j < closestMatches.size(); j++) {
            std::vector<float> rot = GetRotInPlane(kf, closestMatches[j]);
            std::sort(rot.begin(), rot.end());
            // 0 rotation for kf pair without covisibility
            float medianRot = 0;
            if (rot.size() > 0)
                medianRot = rot[(rot.size() - 1) / 2];
            rotIs.insert(std::map<ORB_SLAM2::KeyFrame *, float>::value_type(closestMatches[j], medianRot));
        }

        float max_depth;
        float min_depth;
        // get max_depth and min_depth in current key frame to limit search range
        StereoSearchConstraints(kf, &min_depth, &max_depth);

        cv::Mat image = kf->GetImage();
        cv::Mat image_debug = image.clone();

        std::vector<cv::Mat> F;
        F.clear();
        for (size_t j = 0; j < closestMatches.size(); j++) {
            ORB_SLAM2::KeyFrame *kf2 = closestMatches[j];
            cv::Mat F12 = ComputeFundamental(kf, kf2);
            F.push_back(F12);
        }

#pragma omp parallel for schedule(dynamic) collapse(2)
        for (int y = 0 + 2; y < image.rows - 2; y++) {
            for (int x = 0 + 2; x < image.cols - 2; x++) {

                if (kf->GradImg.at<float>(y, x) < lambdaG) { continue; }
                float pixel = (float) image.at<uchar>(y, x); //maybe it should be cv::Mat

                std::vector<depthHo> depth_ho;
                depth_ho.clear();
                for (size_t j = 0; j < closestMatches.size(); j++) {
                    ORB_SLAM2::KeyFrame *kf2 = closestMatches[j];
                    cv::Mat F12 = F[j];

                    float rot = rotIs[kf2];
                    float best_u(0.0), best_v(0.0);
                    depthHo dh;
                    EpipolarSearch(kf, kf2, x, y, pixel, min_depth, max_depth, &dh, F12, best_u, best_v,
                                   kf->GradTheta.at<float>(y, x), rot);

                    if (dh.supported && 1 / dh.depth > 0.0) {
                        depth_ho.push_back(dh);
                    }
                }

                if (depth_ho.size() > lambdaN) {
                    depthHo dh_temp;
                    InverseDepthHypothesisFusion(depth_ho, dh_temp);
                    if (dh_temp.supported) {
                        kf->depth_map_.at<float>(y, x) = dh_temp.depth;   //  used to do IntraKeyFrameDepthChecking
                        kf->depth_sigma_.at<float>(y, x) = dh_temp.sigma;
                    }
                }

            }
        }
        //cv::imwrite("grad2_image.png",image2_debug);
        // cv::imwrite("depth_image.png",depth_image);
        // saveMatToCsv(depth_image,"depth.csv");

        std::cout << "IntraKeyFrameDepthChecking " << i << std::endl;
        IntraKeyFrameDepthChecking(kf->depth_map_, kf->depth_sigma_, kf->GradImg);
        IntraKeyFrameDepthGrowing(kf->depth_map_, kf->depth_sigma_, kf->GradImg);

//        UpdateSemiDensePointSet(kf);

//        ApplySigmaThreshold(kf);

        kf->semidense_flag_ = true;

        // timing
        clock_gettime(CLOCK_MONOTONIC, &finish);
        duration = (finish.tv_sec - start.tv_sec);
        duration += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
        std::cout << "depth reconstruction took: " << duration << "s" << std::endl;

        //cv::imwrite("image.png",image);
        //pcl::io::savePLYFileBinary ("kf.ply", *cloudPtr);
        //cv::imwrite("grad_image.png",image_debug);

    }


    int num_inter_checked = 0;
    int num_all_neigbors = 0;

    for(size_t i =0;i < vpKFs.size(); i++ )
    {
        ORB_SLAM2::KeyFrame* kf = vpKFs[i];
        if (kf->isBad() || kf->interKF_depth_flag_)continue;

        if (!kf->semidense_flag_) continue;

        // option1: could just be the best covisibility keyframes
//        std::vector<ORB_SLAM2::KeyFrame*> neighbors = kf->GetBestCovisibilityKeyFrames(covisN);
//        if(neighbors.size() < covisN) {continue;}

        std::vector<ORB_SLAM2::KeyFrame*> neighbors;
        std::vector<ORB_SLAM2::KeyFrame*> neighborsAll = kf->GetVectorCovisibleKeyFrames();
        for (size_t idxCov = 0; idxCov < neighborsAll.size(); idxCov++){
            if (neighbors.size() >= covisN)
                break;
            if (neighborsAll[idxCov]->isBad()) continue;
            if (!neighborsAll[idxCov]->Mapped()) continue;
            neighbors.push_back(neighborsAll[idxCov]);
        }
        if(neighbors.size() < covisN) {continue;}


        // start timing
        struct timespec start, finish;
        double duration;
        clock_gettime(CLOCK_MONOTONIC, &start);


        // neighbors have depth reconstructed
        int num_depth_kf = 0;
        for (size_t j = 0; j < neighbors.size(); j++){
            if (neighbors[j]->semidense_flag_) {num_depth_kf++;}
        }

        num_inter_checked++;
        if (num_depth_kf < covisN) continue;
        num_all_neigbors++;

        std::cout << "InterKeyFrameDepthChecking " << i << std::endl;
        InterKeyFrameDepthChecking(kf,neighbors);

        UpdateSemiDensePointSet(kf);

        kf->interKF_depth_flag_ = true;


        //timing
        clock_gettime(CLOCK_MONOTONIC, &finish);
        duration = ( finish.tv_sec - start.tv_sec );
        duration += ( finish.tv_nsec - start.tv_nsec ) / 1000000000.0;
        std::cout<<"depth checking took: "<< duration << "s" << std::endl;

    }


}


void ProbabilityMapping::UpdateAllSemiDensePointSet(){
    vector<ORB_SLAM2::KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    if(vpKFs.size() < 10){return;}

    for (size_t i = 0; i < vpKFs.size(); i++){
        ORB_SLAM2::KeyFrame* kf = vpKFs[i];
        if (kf->isBad() || !kf->interKF_depth_flag_)
            continue;
        if (kf->poseChanged) {
            UpdateSemiDensePointSet(kf);
            kf->poseChanged = false;
        }
    }
}


void ProbabilityMapping::UpdateSemiDensePointSet(ORB_SLAM2::KeyFrame* kf){

#pragma omp parallel for schedule(dynamic) collapse(2)
    for(int y = 0+2; y < kf->im_.rows-2; y++)
    {
        for(int x = 0+2; x< kf->im_.cols-2; x++)
        {

            if (kf->depth_map_.at<float>(y,x) < 0.000001) {
                kf->SemiDensePointSets_.at<float>(y,3*x+0) = 0.0;
                kf->SemiDensePointSets_.at<float>(y,3*x+1) = 0.0;
                kf->SemiDensePointSets_.at<float>(y,3*x+2) = 0.0;
                continue;
            }

            float inv_d = kf->depth_map_.at<float>(y,x);
            float Z = 1/inv_d ;
            float X = Z *(x- kf->cx ) / kf->fx;
            float Y = Z *(y- kf->cy ) / kf->fy;

            cv::Mat Pc = (cv::Mat_<float>(4,1) << X, Y , Z, 1); // point in camera frame.
            cv::Mat Twc = kf->GetPoseInverse();
            cv::Mat pos = Twc * Pc;

            kf->SemiDensePointSets_.at<float>(y,3*x+0) = pos.at<float>(0);
            kf->SemiDensePointSets_.at<float>(y,3*x+1) = pos.at<float>(1);
            kf->SemiDensePointSets_.at<float>(y,3*x+2) = pos.at<float>(2);
        }
    }

}


void ProbabilityMapping::StereoSearchConstraints(ORB_SLAM2::KeyFrame* kf, float* min_depth, float* max_depth){
    std::vector<float> orb_depths = kf->GetAllPointDepths();

    float sum = std::accumulate(orb_depths.begin(), orb_depths.end(), 0.0);
    float mean = sum / orb_depths.size();

    std::vector<float> diff(orb_depths.size());
    std::transform(orb_depths.begin(), orb_depths.end(), diff.begin(), std::bind2nd(std::minus<float>(), mean));
    float variance = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0)/orb_depths.size();
    float stdev = std::sqrt(variance);

    *max_depth = 1/(mean + 2 * stdev);
    *min_depth = 1/(mean - 2 * stdev);
}

void ProbabilityMapping::EpipolarSearch(ORB_SLAM2::KeyFrame* kf1, ORB_SLAM2::KeyFrame *kf2, const int x, const int y, float pixel,
                                        float min_depth, float max_depth, depthHo *dh,cv::Mat F12,float& best_u,float& best_v,float th_pi,float rot)
{

    float a = x*F12.at<float>(0,0)+y*F12.at<float>(1,0)+F12.at<float>(2,0);
    float b = x*F12.at<float>(0,1)+y*F12.at<float>(1,1)+F12.at<float>(2,1);
    float c = x*F12.at<float>(0,2)+y*F12.at<float>(1,2)+F12.at<float>(2,2);

    if((a/b)< -4 || a/b> 4) return;   // if epipolar direction is approximate to perpendicular, we discard it.  May be product wrong match.

    float old_err = 1000000.0;
    float best_photometric_err = 0.0;
    float best_gradient_modulo_err = 0.0;
    int best_pixel = 0;

    int vj,uj_plus,vj_plus,uj_minus,vj_minus;
    float g, q,denomiator ,ustar , ustar_var;

    float umin(0.0),umax(0.0);
    GetSearchRange(umin,umax,x,y,min_depth,max_depth,kf1,kf2);
    for(int uj = std::ceil(umin); uj <= std::floor(umax); uj++)
    {
        vj =-(int)( (a/b)*uj+(c/b));
        if(vj<=0 || vj >= kf2->im_.rows ){continue;}

        // condition 1:
        if( kf2->GradImg.at<float>(vj,uj) < lambdaG){continue;}

        // condition 2:
        float th_epipolar_line = cv::fastAtan2(-a/b,1);
        float temp_gradth =  kf2->GradTheta.at<float>(vj,uj);
        float ang_diff = temp_gradth - th_epipolar_line;
        if(ang_diff >= 360) { ang_diff -= 360; }
        if(ang_diff < 0) { ang_diff += 360; }
        if(ang_diff > 180) ang_diff = 360 - ang_diff;
        if(ang_diff > 90) ang_diff = 180 - ang_diff;
        if(ang_diff > lambdaL) { continue; }

        // condition 3:
        float ang_pi_rot = th_pi + rot;
        if(ang_pi_rot >= 360) { ang_pi_rot -= 360; }
        if(ang_pi_rot < 0) { ang_pi_rot += 360; }
        float th_diff = kf2->GradTheta.at<float>(vj,uj) - ang_pi_rot;
        if(th_diff >= 360) { th_diff -= 360; }
        if(th_diff < 0) { th_diff += 360; }
        if(th_diff > 180) th_diff = 360 - th_diff;
        if(th_diff > lambdaTheta) continue;

        float photometric_err = pixel - bilinear<uchar>(kf2->im_,-((a/b)*uj+(c/b)),uj);
        float gradient_modulo_err = kf1->GradImg.at<float>(y,x)  - bilinear<float>( kf2->GradImg,-((a/b)*uj+(c/b)),uj);

        float err = (photometric_err*photometric_err  + (gradient_modulo_err*gradient_modulo_err)/THETA);
        if(err < old_err)
        {
            best_pixel = uj;
            old_err = err;
            best_photometric_err = photometric_err;
            best_gradient_modulo_err = gradient_modulo_err;
        }
    }

    if(old_err < 1000000.0)
    {

        uj_plus = best_pixel + 1;
        uj_minus = best_pixel - 1;

        g = (bilinear<uchar>(kf2->im_,-((a/b)*uj_plus+(c/b)),uj_plus) - bilinear<uchar>(kf2->im_,-((a/b)*uj_minus+(c/b)),uj_minus)) / 2;
        q = (bilinear<float>(kf2->GradImg,-((a/b)*uj_plus+(c/b)),uj_plus) -  bilinear<float>(kf2->GradImg,-((a/b)*uj_minus+(c/b)),uj_minus)) / 2;

        denomiator = (g*g + (1/THETA)*q*q);
        ustar = best_pixel + (g*best_photometric_err + (1/THETA)*q*best_gradient_modulo_err)/denomiator;
        ustar_var = (2*kf2->I_stddev*kf2->I_stddev/denomiator);

        best_u = ustar;
        best_v =  -( (a/b)*best_u + (c/b) );

        ComputeInvDepthHypothesis(kf1, kf2, ustar, ustar_var, a, b, c, dh,x,y);
    }

}

std::vector<float> ProbabilityMapping::GetRotInPlane(ORB_SLAM2::KeyFrame* kf1, ORB_SLAM2::KeyFrame* kf2){
    std::vector<ORB_SLAM2::MapPoint*> vMPs1 = kf1->GetMapPointMatches();
    std::vector<ORB_SLAM2::MapPoint*> vMPs2 = kf2->GetMapPointMatches();
    std::vector<float> rotInPlane;
    for (size_t idx1 = 0; idx1 < vMPs1.size(); idx1++){
        if (vMPs1[idx1]){
            for (size_t idx2 = 0; idx2 < vMPs2.size(); idx2++) {
                if (vMPs2[idx2] == vMPs1[idx1]) {
                    float angle1 = kf1->GetKeyPointsUn()[idx1].angle;
                    float angle2 = kf2->GetKeyPointsUn()[idx2].angle;
                    if (angle1 < 0 || angle2 < 0) continue;
                    rotInPlane.push_back(angle2 - angle1);
                }
            }
        }
    }
    return rotInPlane;
}

void ProbabilityMapping::IntraKeyFrameDepthChecking(cv::Mat& depth_map, cv::Mat& depth_sigma,const cv::Mat gradimg)
{
    cv::Mat depth_map_new = depth_map.clone();
    cv::Mat depth_sigma_new = depth_sigma.clone();

#pragma omp parallel for schedule(dynamic) collapse(2)
    for (int py = 2; py < (depth_map.rows - 2); py++)
    {
        for (int px = 2; px < (depth_map.cols - 2); px++)
        {

            if (depth_map.at<float>(py,px) > 0.000001)  // if  d !=0.0
            {
                std::vector<depthHo> compatible_neighbor_ho;

                depthHo dha,dhb;
                dha.depth = depth_map.at<float>(py,px);
                dha.sigma = depth_sigma.at<float>(py,px);
                for (int y = py - 1; y <= py + 1; y++)
                {
                    for (int x = px - 1; x <= px + 1; x++)
                    {

                        if (x == px && y == py) continue;
                        if( depth_map.at<float>(y,x) > 0.000001)
                        {
                            if(ChiTest(depth_map.at<float>(y,x),depth_map.at<float>(py,px),depth_sigma.at<float>(y,x),depth_sigma.at<float>(py,px)))
                            {
                                dhb.depth = depth_map.at<float>(y,x);
                                dhb.sigma = depth_sigma.at<float>(y,x);
                                compatible_neighbor_ho.push_back(dhb);
                            }

                        }
                    }
                }
                compatible_neighbor_ho.push_back(dha);  // dont forget itself.

                if (compatible_neighbor_ho.size() >= 3)
                {
                    depthHo fusion;
                    float min_sigma = 0;
                    GetFusion(compatible_neighbor_ho, fusion, &min_sigma);

                    depth_map_new.at<float>(py,px) = fusion.depth;
                    depth_sigma_new.at<float>(py,px) = min_sigma;

                } else
                {
                    depth_map_new.at<float>(py,px) = 0.0;
                    depth_sigma_new.at<float>(py,px) = 0.0;

                }

            }
        }
    }

    depth_map = depth_map_new.clone();
    depth_sigma = depth_sigma_new.clone();

}

void ProbabilityMapping::IntraKeyFrameDepthGrowing(cv::Mat& depth_map, cv::Mat& depth_sigma,const cv::Mat gradimg)
{
    cv::Mat depth_map_new = depth_map.clone();
    cv::Mat depth_sigma_new = depth_sigma.clone();

#pragma omp parallel for schedule(dynamic) collapse(2)
    for (int py = 2; py < (depth_map.rows - 2); py++)
    {
        for (int px = 2; px < (depth_map.cols - 2); px++)
        {

            if (depth_map.at<float>(py,px) < 0.000001)  // if  d ==0.0 : grow the reconstruction getting more density
            {
                if(gradimg.at<float>(py,px)<lambdaG) continue;
                //search supported  by at least 2 of its 8 neighbours pixels
                std::vector< std::pair<float,float> > supported;

                for( int  y = py - 1 ; y <= py+1; y++)
                    for( int  x = px - 1 ; x <= px+1; x++)
                    {
                        if(x == px && y == py) continue;

                        if(ChiTest(depth_map.at<float>(y,x),depth_map.at<float>(py,px),depth_sigma.at<float>(y,x),depth_sigma.at<float>(py,px)))
                        {
                            std::pair<float, float> depth;
                            depth.first = depth_map.at<float>(y,x);
                            depth.second = depth_sigma.at<float>(y,x);
                            supported.push_back(depth);
                        }

                    }

                if(supported.size() >= 2)
                {
                    float d(0.0),s(0.0);
                    GetFusion(supported,d,s);
                    depth_map_new.at<float>(py,px) = d;
                    depth_sigma_new.at<float>(py,px) = s;
                }
            }

        }
    }

    depth_map = depth_map_new.clone();
    depth_sigma = depth_sigma_new.clone();

}

void ProbabilityMapping::InverseDepthHypothesisFusion(const std::vector<depthHo>& h, depthHo& dist) {
    dist.depth = 0;
    dist.sigma = 0;
    dist.supported = false;

    std::vector<depthHo> compatible_ho;
    std::vector<depthHo> compatible_ho_temp;
    float chi = 0;

    for (size_t a=0; a < h.size(); a++) {

        compatible_ho_temp.clear();
        for (size_t b=0; b < h.size(); b++)
        {
            if (ChiTest(h[a], h[b], &chi))
            {compatible_ho_temp.push_back(h[b]);}// test if the hypotheses a and b are compatible
        }

        if (compatible_ho_temp.size() > compatible_ho.size()){
            compatible_ho.clear();
            compatible_ho.insert(compatible_ho.end(),compatible_ho_temp.begin(),compatible_ho_temp.end());
        }
    }

    // calculate the parameters of the inverse depth distribution by fusing hypotheses
    if (compatible_ho.size() >= lambdaN) {
        GetFusion(compatible_ho, dist, &chi);
    }
}

void ProbabilityMapping::InterKeyFrameDepthChecking(ORB_SLAM2::KeyFrame* currentKf, std::vector<ORB_SLAM2::KeyFrame*> neighbors) {

    // for each pixel of keyframe_i, project it onto each neighbor keyframe keyframe_j
    // and propagate inverse depth

    std::vector <cv::Mat> Rji,tji;
    for(size_t j=0; j<neighbors.size(); j++)
    {
        ORB_SLAM2::KeyFrame* kf2 = neighbors[ j ];

        cv::Mat Rcw1 = currentKf->GetRotation();
        cv::Mat tcw1 = currentKf->GetTranslation();
        cv::Mat Rcw2 = kf2->GetRotation();
        cv::Mat tcw2 = kf2->GetTranslation();

        cv::Mat R21 = Rcw2*Rcw1.t();
        cv::Mat t21 = -Rcw2*Rcw1.t()*tcw1+tcw2;

        Rji.push_back(R21);
        tji.push_back(t21);

    }

    int cols = currentKf->im_.cols;
    int rows = currentKf->im_.rows;
    float fx = currentKf->fx;
    float fy = currentKf->fy;
    float cx = currentKf->cx;
    float cy = currentKf->cy;

#pragma omp parallel for schedule(dynamic) collapse(2)
    for (int py = 2; py <rows-2; py++) {
        for (int px = 2; px < cols-2; px++) {

            if (currentKf->depth_map_.at<float>(py,px) < 0.000001) continue;   //  if d == 0.0  continue;

            float depthp = currentKf->depth_map_.at<float>(py,px);
            // count of neighboring keyframes in which there is at least one compatible pixel
            int compatible_neighbor_keyframes_count = 0;

            // keep track of compatible pixels for the gauss-newton step
            std::vector<std::vector<depthHo>> compatible_pixels_by_frame;
            int num_compatible_pixels = 0;

            for(size_t j=0; j<neighbors.size(); j++) {

                ORB_SLAM2::KeyFrame* pKFj = neighbors[j];
                cv::Mat K = pKFj->GetCalibrationMatrix();

                cv::Mat xp=(cv::Mat_<float>(3,1) << (px-cx)/fx, (py-cy)/fy,1.0);// inverse project.    if has distortion, this code shoud fix
                cv::Mat temp = Rji[j] * xp /depthp + tji[j];
                cv::Mat Xj = K*temp;
                Xj = Xj/Xj.at<float>(2);   //   u = u'/z   ,  v = v'/z

                // Eq (12)
                // compute the projection matrix to map 3D point from original image to 2D point in neighbor keyframe
                temp = Rji[j].row(2) * xp;
                float denom1 = temp.at<float>(0,0);
                temp = depthp * tji[j].at<float>(2);
                float denom2 = temp.at<float>(0,0);
                float depthj = depthp / (denom1 + denom2);

                float xj = Xj.at<float>(0);
                float yj = Xj.at<float>(1);

                // look in 4-neighborhood pixel p_j,n around xj for compatible inverse depth
                std::vector<depthHo> compatible_pixels_J;
                if (xj<0 || xj>=cols-1 || yj<0 || yj>=rows-1) {
                    // make sure kf j correspondence
                    compatible_pixels_by_frame.push_back(compatible_pixels_J);
                    continue;
                }
                int x0 = (int)std::floor(xj);
                int y0 = (int )std::floor(yj);
                int x1 = x0 + 1;
                int y1 =  y0 + 1;

                float d = pKFj->depth_map_.at<float>(y0,x0);
                float sigma = pKFj->depth_sigma_.at<float>(y0,x0);
                if(d>0.000001)
                {
                    float test = pow((depthj - d),2)/pow(sigma,2);
                    if (test < 3.84) {
                        depthHo dHo;
                        dHo.depth = d;
                        dHo.sigma = sigma;
                        compatible_pixels_J.push_back(dHo);
                    }
                }
                d = pKFj->depth_map_.at<float>(y1,x0);
                sigma = pKFj->depth_sigma_.at<float>(y1,x0);
                if(d>0.000001)
                {
                    float test = pow((depthj - d),2)/pow(sigma,2);
                    if (test < 3.84) {
                        depthHo dHo;
                        dHo.depth = d;
                        dHo.sigma = sigma;
                        compatible_pixels_J.push_back(dHo);
                    }
                }
                d = pKFj->depth_map_.at<float>(y0,x1);
                sigma = pKFj->depth_sigma_.at<float>(y0,x1);
                if(d>0.000001)
                {
                    float test = pow((depthj - d),2)/pow(sigma,2);
                    if (test < 3.84) {
                        depthHo dHo;
                        dHo.depth = d;
                        dHo.sigma = sigma;
                        compatible_pixels_J.push_back(dHo);
                    }
                }
                d = pKFj->depth_map_.at<float>(y1,x1);
                sigma = pKFj->depth_sigma_.at<float>(y1,x1);
                if(d>0.000001)
                {
                    float test = pow((depthj - d),2)/pow(sigma,2);
                    if (test < 3.84) {
                        depthHo dHo;
                        dHo.depth = d;
                        dHo.sigma = sigma;
                        compatible_pixels_J.push_back(dHo);
                    }
                }

                // at least one compatible pixel p_j,n must be found in at least lambdaN neighbor keyframes
                if (compatible_pixels_J.size() >= 1) {compatible_neighbor_keyframes_count++;}
                compatible_pixels_by_frame.push_back(compatible_pixels_J);
                num_compatible_pixels += compatible_pixels_J.size();

            } // for j = 0...neighbors.size()-1

            // don't retain the inverse depth distribution of this pixel if not enough support in neighbor keyframes
            if (compatible_neighbor_keyframes_count < lambdaN )
            {
                currentKf->depth_map_.at<float>(py,px) = 0.0;
            }
            else {
                // gauss newton smoothing
                cv::Mat xp=(cv::Mat_<float>(3,1) << (px-cx)/fx, (py-cy)/fy,1.0);
                float dp = 1/depthp;

                cv::Mat J = cv::Mat(num_compatible_pixels,1,CV_32F);
                cv::Mat r0 = cv::Mat(num_compatible_pixels,1,CV_32F);
                int idxJN = 0;
                for (size_t j = 0; j < compatible_pixels_by_frame.size(); j++){
                    std::vector<depthHo>& compatibleJ = compatible_pixels_by_frame[j];
                    for (size_t n = 0; n < compatibleJ.size(); n++){
                        cv::Mat rzxp = Rji[j].row(2) * xp;
                        float djn = 1/compatibleJ[n].depth;
                        float sigmajn = compatibleJ[n].sigma;
                        float d2sigma = djn*djn * sigmajn;

                        J.at<float>(idxJN,0) = - rzxp.at<float>(0,0) / d2sigma;
                        r0.at<float>(idxJN,0) = (djn - dp*rzxp.at<float>(0,0) - tji[j].at<float>(2,0)) / d2sigma;

                        idxJN++;
                    }
                }
                cv::Mat Jtr0 = - J.t() * r0;
                cv::Mat JtJ = J.t() * J;

                float dpDelta = Jtr0.at<float>(0,0) / JtJ.at<float>(0,0);

                currentKf->depth_map_.at<float>(py,px) = 1/ (dp + dpDelta);
            }

        } // for py = 0...im.cols-1
    } // for px = 0...im.rows-1

}


////////////////////////
// Utility functions
////////////////////////

void ProbabilityMapping::ComputeInvDepthHypothesis(ORB_SLAM2::KeyFrame* kf, ORB_SLAM2::KeyFrame* kf2, float ustar, float ustar_var,
                                                   float a, float b, float c,ProbabilityMapping::depthHo *dh, int x,int y) {

    float inv_pixel_depth =  0.0;

    // equation 8 comput depth
    GetPixelDepth(ustar, x , y,kf, kf2,inv_pixel_depth);

    float ustar_min = ustar - sqrt(ustar_var);
    float inv_depth_min = 0.0;
    GetPixelDepth(ustar_min,x,y,kf,kf2, inv_depth_min);

    float ustar_max = ustar +  sqrt(ustar_var);
    float inv_depth_max = 0.0;
    GetPixelDepth(ustar_max,x,y,kf, kf2,inv_depth_max);

    // Equation 9
    float sigma_depth = cv::max(abs(inv_depth_max-inv_pixel_depth), abs(inv_depth_min-inv_pixel_depth));

    dh->depth = inv_pixel_depth;
    dh->sigma = sigma_depth;
    dh->supported = true;

}

//Xp = K-1 * xp (below Equation 8)
// map 2D pixel coordinate to 3D point
void ProbabilityMapping::GetXp(const cv::Mat& k, int px, int py, cv::Mat* xp) {

    cv::Mat xp2d = cv::Mat(3,1,CV_32F);

    xp2d.at<float>(0,0) = px;
    xp2d.at<float>(1,0) = py;
    xp2d.at<float>(2,0) = 1;

    *xp = k.inv() * xp2d;
}

// Equation (8)
void ProbabilityMapping::GetPixelDepth(float uj, int px, int py, ORB_SLAM2::KeyFrame* kf,ORB_SLAM2::KeyFrame* kf2, float &p) {

    float fx = kf->fx;
    float cx = kf->cx;
    float fy = kf->fy;
    float cy = kf->cy;

    float ucx = uj - cx;

    cv::Mat Rcw1 = kf->GetRotation();
    cv::Mat tcw1 = kf->GetTranslation();
    cv::Mat Rcw2 = kf2->GetRotation();
    cv::Mat tcw2 = kf2->GetTranslation();

    cv::Mat R21 = Rcw2*Rcw1.t();
    cv::Mat t21 = -Rcw2*Rcw1.t()*tcw1+tcw2;

    cv::Mat xp=(cv::Mat_<float>(3,1) << (px-cx)/fx, (py-cy)/fy,1.0);// inverse project.    if has distortion, this code shoud fix

    //GetXp(kf->GetCalibrationMatrix(), px, py, &xp);

    cv::Mat temp = R21.row(2) * xp * ucx;
    float num1 = temp.at<float>(0,0);
    temp = fx * (R21.row(0) * xp);
    float num2 = temp.at<float>(0,0);
    float denom1 = -t21.at<float>(2) * ucx;
    float denom2 = fx * t21.at<float>(0);

    p = (num1 - num2) / (denom1 + denom2);

}

void ProbabilityMapping::GetSearchRange(float& umin, float& umax, int px, int py,float mind,float maxd,
                                        ORB_SLAM2::KeyFrame* kf,ORB_SLAM2::KeyFrame* kf2)
{
    float fx = kf->fx;
    float cx = kf->cx;
    float fy = kf->fy;
    float cy = kf->cy;

    cv::Mat Rcw1 = kf->GetRotation();
    cv::Mat tcw1 = kf->GetTranslation();
    cv::Mat Rcw2 = kf2->GetRotation();
    cv::Mat tcw2 = kf2->GetTranslation();

    cv::Mat R21 = Rcw2*Rcw1.t();
    cv::Mat t21 = -Rcw2*Rcw1.t()*tcw1+tcw2;

    cv::Mat xp1=(cv::Mat_<float>(3,1) << (px-cx)/fx, (py-cy)/fy,1.0);  // inverse project.    if has distortion, this code shoud fix
    cv::Mat xp2_min = R21*xp1*mind+t21;
    cv::Mat xp2_max = R21*xp1*maxd+t21;

    umin = fx*xp2_min.at<float>(0)/xp2_min.at<float>(2) + cx;
    umax = fx*xp2_max.at<float>(0)/xp2_max.at<float>(2) + cx;

    if (umin > umax){
        float temp = umax;
        umax = umin;
        umin = temp;
    }

    if(umin<0) umin = 0;
    if(umax<0) umax = 0;
    if(umin>kf->im_.cols ) umin = kf->im_.cols;
    if(umax>kf->im_.cols)  umax = kf->im_.cols;
}

bool ProbabilityMapping::ChiTest(const depthHo& ha, const depthHo& hb, float* chi_val) {
    float num = (ha.depth - hb.depth)*(ha.depth - hb.depth);
    float chi_test = num / (ha.sigma*ha.sigma) + num / (hb.sigma*hb.sigma);
    if (chi_val)
        *chi_val = chi_test;
    return (chi_test < 5.99);  // 5.99 -> 95%
}

bool ProbabilityMapping::ChiTest(const float& a, const float& b, const float sigma_a,float sigma_b) {
    float num = (a - b)*(a - b);
    float chi_test = num / (sigma_a*sigma_a) + num / (sigma_b*sigma_b);
    return (chi_test < 5.99);  // 5.99 -> 95%
}

void ProbabilityMapping::GetFusion(const std::vector<std::pair <float,float> > supported, float& depth, float& sigma)
{
    size_t t = supported.size();
    float pjsj =0; // numerator
    float rsj =0; // denominator

    float min_sigma = supported[0].second;

    for(size_t i = 0; i< t; i++)
    {
        pjsj += supported[i].first / pow(supported[i].second, 2);
        rsj += 1 / pow(supported[i].second, 2);

        if (supported[i].second < min_sigma)
            min_sigma = supported[i].second;
    }

    depth = pjsj / rsj;
    sigma = min_sigma;
}

void ProbabilityMapping::GetFusion(const std::vector<depthHo>& compatible_ho, depthHo& hypothesis, float* min_sigma) {
    hypothesis.depth = 0;
    hypothesis.sigma = 0;

    float temp_min_sigma = compatible_ho[0].sigma;
    float pjsj =0; // numerator
    float rsj =0; // denominator

    for (size_t j = 0; j < compatible_ho.size(); j++) {
        pjsj += compatible_ho[j].depth / pow(compatible_ho[j].sigma, 2);
        rsj += 1 / pow(compatible_ho[j].sigma, 2);
        if (pow(compatible_ho[j].sigma, 2) < pow(temp_min_sigma, 2)) {
            temp_min_sigma = compatible_ho[j].sigma;
        }
    }

    hypothesis.depth = pjsj / rsj;
    hypothesis.sigma = sqrt(1 / rsj);
    hypothesis.supported = true;

    if (min_sigma) {
        *min_sigma = temp_min_sigma;
    }
}

cv::Mat ProbabilityMapping::ComputeFundamental( ORB_SLAM2::KeyFrame *&pKF1,  ORB_SLAM2::KeyFrame *&pKF2) {
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = GetSkewSymmetricMatrix(t12);

    cv::Mat K1 = pKF1->GetCalibrationMatrix();
    cv::Mat K2 = pKF2->GetCalibrationMatrix();

    return K1.t().inv()*t12x*R12*K2.inv();
}
