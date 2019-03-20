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
 *        Log: fix a lot of bug, Almost rewrite the code
 *
 *        author: He Yijia
 *
 *        Version: 1.1
 *        Created: 05/18/2017
 *        Author: Shida He
 *
 * =====================================================================================
 */

#ifndef PROBABILITYMAPPING_H
#define PROBABILITYMAPPING_H

#include <cstdlib>
#include <stdio.h>
#include <vector>
#include <numeric>
#include <Eigen/Core>
#include <mutex>

#define covisN 7
#define sigmaI 20
#define lambdaG 8//8
#define lambdaL 80
#define lambdaTheta 45  // 45
#define lambdaN 3
#define histo_length 30
#define th_high 100
#define th_low 50
#define THETA 0.23
#define NNRATIO 0.6
#define NULL_DEPTH 999


namespace ORB_SLAM2 {
    class KeyFrame;
    class Map;
}

namespace cv {
    class Mat;
}

class ProbabilityMapping {
public:

    struct depthHo {
        depthHo():depth(0.0),sigma(0.0),supported(false),Pw(0.0, 0.0, 0.0){};
        float depth;
        float sigma;
        bool supported;
        Eigen::Vector3f Pw; // point pose in world frame
    };

    ProbabilityMapping(ORB_SLAM2::Map *pMap);

    /* * \brief void SemiDenseLoop(ORB_SLAM2::KeyFrame kf, depthHo**, std::vector<depthHo>*): return results of epipolar search (depth hypotheses) */
    void SemiDenseRecon(ORB_SLAM2::KeyFrame* kf);
    /* * \brief void stereo_search_constraints(): return min, max inverse depth */
    void StereoSearchConstraints(ORB_SLAM2::KeyFrame* kf, float* min_depth, float* max_depth);
    /* * \brief void epipolar_search(): return distribution of inverse depths/sigmas for each pixel */
    void EpipolarSearch(ORB_SLAM2::KeyFrame *kf1, ORB_SLAM2::KeyFrame *kf2, const int x, const int y, float pixel, float min_depth, float max_depth, depthHo *dh, cv::Mat F12, float &best_u, float &best_v,float th_pi,float rot);
    void GetSearchRange(float& umin, float& umax, int px, int py, float mind, float maxd, ORB_SLAM2::KeyFrame* kf, ORB_SLAM2::KeyFrame* kf2);
    /* * \brief void inverse_depth_hypothesis_fusion(const vector<depthHo> H, depthHo* dist):
 * *         get the parameters of depth hypothesis distrubution from list of depth hypotheses */
    void InverseDepthHypothesisFusion(const std::vector<depthHo>& h, depthHo &dist);
    //  vector is low.  use depth_map and detph_sigma (cv::Mat)  to speed
    void IntraKeyFrameDepthChecking(cv::Mat& depth_map, cv::Mat& depth_sigma, const cv::Mat gradimg);
    void IntraKeyFrameDepthGrowing(cv::Mat& depth_map, cv::Mat& depth_sigma, const cv::Mat gradimg);

    void UpdateSemiDensePointSet(ORB_SLAM2::KeyFrame* kf);
    void UpdateAllSemiDensePointSet();

    void InterKeyFrameDepthChecking(ORB_SLAM2::KeyFrame* currentKf, std::vector<ORB_SLAM2::KeyFrame*> neighbors);


private:
    void ComputeInvDepthHypothesis(ORB_SLAM2::KeyFrame* kf, ORB_SLAM2::KeyFrame *kf2, float ustar, float ustar_var, float a, float b, float c, depthHo *dh, int x, int y);
    void GetPixelDepth(float uj, int px, int py, ORB_SLAM2::KeyFrame* kf, ORB_SLAM2::KeyFrame *kf2, float &p);
    bool ChiTest(const depthHo& ha, const depthHo& hb, float* chi_val);
    bool ChiTest(const float& a, const float& b, const float sigma_a, float sigma_b);
    void GetFusion(const std::vector<std::pair <float,float> > supported, float& depth, float& sigma);
    void GetFusion(const std::vector<depthHo>& best_compatible_ho, depthHo& hypothesis, float* min_sigma);
    cv::Mat ComputeFundamental(ORB_SLAM2::KeyFrame *&pKF1, ORB_SLAM2::KeyFrame *&pKF2);
    cv::Mat GetSkewSymmetricMatrix(const cv::Mat &v);
    std::vector<float> GetRotInPlane(ORB_SLAM2::KeyFrame* kf1, ORB_SLAM2::KeyFrame* kf2);

};

#endif
