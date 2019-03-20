//
// Created by theia on 1/10/17.
//

#ifndef ORB_SLAM2_MODELFRAME_H
#define ORB_SLAM2_MODELFRAME_H

#include "KeyFrame.h"
#include "MapPoint.h"
#include "Frame.h"

namespace ORB_SLAM2 {

    class KeyFrame;
    class MapPoint;
    class Frame;

    class TextureFrame {
    public:
        TextureFrame(KeyFrame *pKF);

        TextureFrame(Frame *pF);

        vector<float> GetTexCoordinate(float x, float y, float z, cv::Size s);

        vector<float> GetTexCoordinate(float x, float y, float z);

        cv::Mat GetOrientation();

        long unsigned int mFrameID;
        cv::Mat mRcw;
        cv::Mat mtcw;
        cv::Mat mTwc;

        float mfx, mfy, mcx, mcy;
        float mnMinX;
        float mnMaxX;
        float mnMinY;
        float mnMaxY;

        KeyFrame* mpKF;
        Frame* mpF;
    };

}

#endif //ORB_SLAM2_MODELFRAME_H
