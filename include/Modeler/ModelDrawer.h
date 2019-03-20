//
// Created by shida on 06/12/16.
//

#ifndef __MODELDRAWER_H
#define __MODELDRAWER_H

#include<pangolin/pangolin.h>

#include<mutex>

#include <map>
#include <list>
#include <deque>
#include <vector>
#include "Modeler/Matrix.h"
#include "Modeler/Modeler.h"
#include "Modeler/TextureFrame.h"

namespace ORB_SLAM2
{

    class KeyFrame;
    class Modeler;

    class ModelDrawer
    {
    public:
        ModelDrawer();

        void DrawModel(bool bRGB);
        void DrawModelPoints();
        void DrawTriangles(pangolin::OpenGlMatrix &Twc);
        void DrawFrame(bool bRGB);
        cv::Mat DrawLines();

        void UpdateModel();
        void SetUpdatedModel(const vector<dlovi::Matrix> & modelPoints, const list<dlovi::Matrix> & modelTris);

        void MarkUpdateDone();
        bool UpdateRequested();
        bool UpdateDone();

        vector<dlovi::Matrix> & GetPoints();
        list<dlovi::Matrix> & GetTris();

        void SetModeler(Modeler* pModeler);
        Modeler* mpModeler;
    private:



        bool mbModelUpdateRequested;
        bool mbModelUpdateDone;

        std::pair<vector<dlovi::Matrix>, list<dlovi::Matrix>> mModel;
        std::pair<vector<dlovi::Matrix>, list<dlovi::Matrix>> mUpdatedModel;

    };

} //namespace ORB_SLAM

#endif //__MODELDRAWER_H
