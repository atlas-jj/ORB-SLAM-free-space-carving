# Code Structure

## Adding map points from ORB-SLAM to CARV model

1. The modeler instance is declared in system.h, line 137.
```c++
        //CARV: Modeler that take map logs to create and display the reconstructed model
        Modeler* mpModeler;
``` 

2. The thread to run CARV model is declared in system.h, line 179.
```c++
         //CARV: Modeler thread
         std::thread* mptModeler;
```
bound to mpModeler via sytem.cc, line 111:
```c++
         mptModeler = new thread(&ORB_SLAM2::Modeler::Run, mpModeler);
```
3. mpModeler is passed to mpMap, mpTracker, mpLocalMapper and mpModelDrawer in system.cc, line 132-135:
```c++
        //CARV: set pointer of modeler
        mpMap->SetModeler(mpModeler);
        mpTracker->SetModeler(mpModeler);
        mpLocalMapper->SetModeler(mpModeler);
        mpModelDrawer->SetModeler(mpModeler);
```
Operations (add, delete, update and get) on the CARV model are defined in the above mentioned class instances.

4. For example, add new points to the CARV model: in LocalMapping.cc, line 94:
```c++
         //CARV: add keyframe to transcript queue
         mpModeler->AddKeyFrameEntry(mpCurrentKeyFrame);
```
  
## Implementation of CARV model
All codes are under /Modeler
1. core algorithm is implemented in FreespaceDelaunayAlgorithm.cc.
2. The interface opened for ORB-SLAM main thread is Modeler.cc. For example:
3D geometric shape based methods:
```c++
        void AddKeyFrameEntry(KeyFrame* pKF);
        void AddDeletePointEntry(MapPoint* pMP);
        void AddDeleteObservationEntry(KeyFrame* pKF, MapPoint* pMP);
        void AddAdjustmentEntry(std::set<KeyFrame*> & sAdjustSet, std::set<MapPoint*> & sMapPoints);
```

Rendering image textures:
```c++
        void AddTexture(KeyFrame* pKF);
        void AddTexture(Frame* pF);
        void AddFrameImage(const long unsigned int &frameID, const cv::Mat &im);
```
Modeler.cc has three key members defined in Modeler.h, line 100-104:
```c++
        //CARV interface
        SFMTranscriptInterface_ORBSLAM mTranscriptInterface; // An interface to a transcript / log of the map's work.
        //CARV runner instance
        dlovi::FreespaceDelaunayAlgorithm mObjAlgorithm;
        SFMTranscriptInterface_Delaunay mAlgInterface; // An encapsulation of the interface between the transcript and the surface inferring algorithm.
```

- **mTranscriptInterface is used to convert ORB-SLAM points to CARV command strings.**
- **mObjAlgorithm is the core algorithm to compute 3D CARV model.**
- **mAlgInterface is used to parse command strings to readable data and invoke mObjAlgorithm to calculate CARV model.**

Initialized in Modeler.cc, line 14-22, using:
```c++
        mAlgInterface.setAlgorithmRef(&mObjAlgorithm);
        mAlgInterface.setTranscriptRef(mTranscriptInterface.getTranscriptToProcessRef());
        mAlgInterface.rewind();
        commandLineEnterBlock = false;
```

3. In order to save and restore 3D CARV models, transcript commands are used. For example, the command transcript for adding new map points in a key frame could be:
```c++
        new cam: [-0.0215062; 0.00396759; -0.00969708] {
          new point: [0.120206; 0.132441; 1.01889], 1, 0
          new point: [-0.525088; -0.238598; 1.04532], 1, 0
          new point: [-0.0281776; 0.498589; 0.961208], 1, 0
          new point: [-0.493396; -0.223343; 0.982285], 1, 0
          new point: [0.0300069; 0.432055; 0.945879], 1, 0
          new point: [-0.0283748; 0.504657; 0.972257], 1, 0
          new point: [-0.520822; -0.0684948; 1.06941], 1, 0
          new point: [-0.502707; -0.15712; 1.01276], 1, 0
          observation: 0
          observation: 1
          observation: 2
          observation: 3
          observation: 4
          observation: 5
          observation: 6
        }
```
4. The interface to parse the above mentioned transcript is SFMTranscript.cpp. For example, line 614-834:
```c++
int SFMTranscript::stepTranscriptBody(EntryType & enumEntryType, EntryData & objEntryData, std::vector<dlovi::Matrix> & arrPoints,std::vector<dlovi::Matrix> & arrCamCenters, std::vector<std::vector<int> > & arrVisLists, int nLoop){

            // TODO: Remove me
            //static std::set<int> setDeletedPoints;
            //static std::set<int> setReaddedPoints;
            // -------------------

            try{
                std::string strCurrentLine = trim(getLine(nLoop));
                
                // TODO: throw exception if we go past the # of lines
                while(strCurrentLine.empty())
                    strCurrentLine = trim(getLine(++nLoop));

                if(strCurrentLine.find("reset") != std::string::npos){
                    // eg: reset
                    prepareForNewEntry_Step(enumEntryType, ET_RESET);

                    arrPoints.clear();
                    arrCamCenters.clear();
                    arrVisLists.clear();
                }
                else if(strCurrentLine.find("new point: ") != std::string::npos){
                    // eg: new point: [x; y; z], KF_ind1, KF_ind2, KF_ind3, ..., KF_indN // vis list optional
                    prepareForNewEntry_Step(enumEntryType, ET_POINTINSERTION);
                    if(strCurrentLine.find(", ") != std::string::npos){
                        // Visibility list is present
                        std::vector<std::string> arrStr = split(strCurrentLine.substr(11), ", "); 
                        ...
                }
                ...
                        
                return nLoop + 1;
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscript", "stepTranscriptBody"); ex2.raise();
            }
        }
```
5. The class to execute one command and update model using FreespaceDelaunayAlgorithm is SFMTranscriptInterface_Delaunay. For example, 
```c++
        // run all stacked commands until all processed.
        void SFMTranscriptInterface_Delaunay::runRemainder(){
            try{
                while(!isDone())
                    step();
            }
            catch(std::exception & ex){
                dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_Delaunay", "runRemainder"); cerr << ex2.what() << endl; //ex2.raise();
            }
        }
```

And in the function step(), it will firstly run an instance of SFMTranscript (m_pTranscript) to parse the string command:
```c++
        stepTranscript(false);
```
And then get command type using:
```c++
        getCurrentEntryType(); //get command type from m_pTranscript
```
get command date using:
```c++
        getCurrentEntryData(); //get command data from m_pTranscript
```
After that, it can invoke methods in FreespaceDelaunayAlgorithm to calculate 3D CARV model. CARV model is represented as a pair of lists in SFMTranscriptInterface_Delaunay.h, line 29
```c++
        std::pair<std::vector<dlovi::Matrix>, std::list<dlovi::Matrix> > getCurrentModel() const;
```

6. SFMTranscriptInterface_ORBSLAM.cc: includes a SFMTranscript member:
```c++
        dlovi::compvis::SFMTranscript m_SFMTranscript;
```
m_SFMTranscript is used to store command strings by:
```c++
        m_SFMTranscript.addLine(ssTmp.str());
```
And shared to other instance by:
```c++
        // Getters
        dlovi::compvis::SFMTranscript * getTranscriptRef();
        dlovi::compvis::SFMTranscript * getTranscriptToProcessRef();
```

## Drawing CARV model
1. Viewer.cc, line 183-197
```c++
        // carv: show model or triangle with light from camera
            if(menuShowModel && menuShowTexture) {
                mpModelDrawer->DrawModel(mbRGB);
            }
            else if (menuShowModel && !menuShowTexture) {
                mpModelDrawer->DrawTriangles(MapTwc);
            }
            else if (!menuShowModel && menuShowTexture) {
                mpModelDrawer->DrawFrame(mbRGB);
            }
            if(menuSaveCARV)
            {
              mpSystem->mpModeler->writeToFile("chris_CARV_Files");
              menuSaveCARV = false;
            }
```

2. Modeler/ModelDrawer, line 13-118, 
CARV model stored in ModelDrawer.h, line 56,57:
```c++
        std::pair<vector<dlovi::Matrix>, list<dlovi::Matrix>> mModel;
        std::pair<vector<dlovi::Matrix>, list<dlovi::Matrix>> mUpdatedModel;
```
CARV model updated via ModelDrawer.cc, line 223 and 238,
```c++
        void ModelDrawer::UpdateModel(){...}
        void ModelDrawer::SetUpdatedModel(const vector<dlovi::Matrix> & modelPoints, const list<dlovi::Matrix> & modelTris){...}
```
Texture data obtained via: 
```c++
        vector<pair<cv::Mat,TextureFrame>> imAndTexFrame = mpModeler->GetTextures(numKFs);
```
Drawing function is: void ModelDrawer::DrawModel(bool bRGB);

3. When to update model?
- In Modeler.cc, line 51, function void Modeler::Run()
```c++
        while(1) {
            if (CheckNewTranscriptEntry()) {

                RunRemainder(); //parse transcript command and calculate model

                UpdateModelDrawer(); //update model, set to model drawer.
            }
            ...
         }
```
- In Modeler.cc, line 1295, 
```c++
        void Modeler::UpdateModelDrawer() {
          if(mpModelDrawer->UpdateRequested() && ! mpModelDrawer->UpdateDone()) {
              std::pair<std::vector<dlovi::Matrix>, std::list<dlovi::Matrix> > objModel = mAlgInterface.getCurrentModel();
              mpModelDrawer->SetUpdatedModel(objModel.first, objModel.second);
              mpModelDrawer->MarkUpdateDone();
          }
        }
```

