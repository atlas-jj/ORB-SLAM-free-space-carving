//
// Created by shida on 14/01/17.
//

#include "Modeler/TextureFrame.h"

namespace ORB_SLAM2 {

    TextureFrame::TextureFrame(KeyFrame *pKF) {
        mpKF = pKF;
        mpF = NULL;

        mFrameID = pKF->mnFrameId;
        // GetPose instead GetPoseInverse, seems camera position need to be inversed
        mRcw = pKF->GetPose().rowRange(0, 3).colRange(0, 3);
        mtcw = pKF->GetPose().rowRange(0, 3).col(3);
        mTwc = pKF->GetPoseInverse();

        mfx = pKF->fx;
        mfy = pKF->fy;
        mcx = pKF->cx;
        mcy = pKF->cy;
        mnMaxX = pKF->mnMaxX;
        mnMinX = pKF->mnMinX;
        mnMaxY = pKF->mnMaxY;
        mnMinY = pKF->mnMinY;
    }

    TextureFrame::TextureFrame(Frame *pF) {
        mpF = pF;
        mpKF = NULL;

        mFrameID = pF->mnId;
        // GetPose instead GetPoseInverse, seems camera position need to be inversed
        mRcw = pF->mTcw.rowRange(0, 3).colRange(0, 3);
        mtcw = pF->mTcw.rowRange(0, 3).col(3);
        mTwc = pF->mTcw.inv();

        mfx = pF->fx;
        mfy = pF->fy;
        mcx = pF->cx;
        mcy = pF->cy;
        mnMaxX = pF->mnMaxX;
        mnMinX = pF->mnMinX;
        mnMaxY = pF->mnMaxY;
        mnMinY = pF->mnMinY;
    }

    cv::Mat TextureFrame::GetOrientation() {
        const cv::Mat AxisZ = (cv::Mat_<float>(3, 1) << 0.0, 0.0, 1.0);

        // 3D in world coordinates
        const cv::Mat Rwc = mTwc.rowRange(0, 3).colRange(0, 3);
        const cv::Mat twc = mTwc.rowRange(0, 3).col(3);

        cv::Mat Orientation = Rwc * AxisZ + twc;
        Orientation = Orientation * (1 / norm(Orientation));

        return Orientation;
    }

    vector<float> TextureFrame::GetTexCoordinate(float x, float y, float z, cv::Size s) {
        const cv::Mat P = (cv::Mat_<float>(3, 1) << x, y, z);
        // 3D in camera coordinates
        const cv::Mat Pc = mRcw * P + mtcw;
        const float &PcX = Pc.at<float>(0);
        const float &PcY = Pc.at<float>(1);
        const float &PcZ = Pc.at<float>(2);

        std::vector<float> uv;
        if(PcZ > 0) {
            // Project in image and check it is not outside
            const float invz = 1.0f / PcZ;
            const float u = mfx * PcX * invz + mcx;
            const float v = mfy * PcY * invz + mcy;

            float uTex = u / s.width;
            float vTex = v / s.height;
            if (uTex > 0 && uTex < 1 && vTex > 0 && vTex < 1) {
                uv.push_back(uTex);
                uv.push_back(vTex);
            }
        }
        return uv;

    }

    vector<float> TextureFrame::GetTexCoordinate(float x, float y, float z) {
        const cv::Mat P = (cv::Mat_<float>(3, 1) << x, y, z);
        // 3D in camera coordinates
        const cv::Mat Pc = mRcw * P + mtcw;
        const float &PcX = Pc.at<float>(0);
        const float &PcY = Pc.at<float>(1);
        const float &PcZ = Pc.at<float>(2);

        std::vector<float> uv;
        if(PcZ > 0) {
            // Project in image and check it is not outside
            const float invz = 1.0f / PcZ;
            const float u = mfx * PcX * invz + mcx;
            const float v = mfy * PcY * invz + mcy;

            float uTex = u / (mnMaxX - mnMinX);
            float vTex = v / (mnMaxY - mnMinY);
            if (uTex > 0 && uTex < 1 && vTex > 0 && vTex < 1) {
                uv.push_back(uTex);
                uv.push_back(vTex);
            }
        }
        return uv;
    }


}