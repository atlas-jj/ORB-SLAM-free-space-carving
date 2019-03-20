#ifndef __SFMTRANSCRIPTINTERFACE_DELAUNAY_H
#define __SFMTRANSCRIPTINTERFACE_DELAUNAY_H

#include "Modeler/SFMTranscript.h"
#include "Modeler/FreespaceDelaunayAlgorithm.h"
#include <vector>
#include <string>
#include <utility>

class SFMTranscriptInterface_Delaunay;

namespace dlovi{
    //namespace compvis{
    //  class SFMTranscript;
    //}
    class FreespaceDelaunayAlgorithm;
    //class FreespaceDelaunayAlgorithm::Delaunay3;
    //class FreespaceDelaunayAlgorithm::Delaunay3::Vertex_handle;
}

class SFMTranscriptInterface_Delaunay{
public:
    // Constructors and Destructors
    SFMTranscriptInterface_Delaunay();
    SFMTranscriptInterface_Delaunay(dlovi::compvis::SFMTranscript * pTranscript, dlovi::FreespaceDelaunayAlgorithm * pAlgorithm);
    ~SFMTranscriptInterface_Delaunay();

    // Getters
    std::pair<std::vector<dlovi::Matrix>, std::list<dlovi::Matrix> > getCurrentModel() const;
    dlovi::compvis::SFMTranscript::EntryType getCurrentEntryType() const;
    const dlovi::compvis::SFMTranscript::EntryData & getCurrentEntryData() const;
    std::string getCurrentEntryText() const;
    std::vector<dlovi::Matrix> getCurrentEntryPoints() const;
    std::vector<dlovi::Matrix> getCurrentEntryCamCenters() const;
    std::vector<std::vector<int> > getCurrentEntryVisList() const;
    int numFreeSpaceConstraintsInTriangulation() const;

    // Setters
    void setTranscriptRef(dlovi::compvis::SFMTranscript * pTranscript);
    void setAlgorithmRef(dlovi::FreespaceDelaunayAlgorithm * pAlgorithm);

    // Public Methods
    void loadTranscriptFromFile(const std::string & strFileName);
    void processTranscript();
    void stepTranscript(bool bFirstEntry = false);
    void runFull();
    void runOnlyFinalState();
    void runRemainder();
    void step(bool bRunAlgorithm = true);
    void rewind();
    bool isDone();
    void writeCurrentModelToFile(const std::string & strFileName) const;

private:
    // Private Methods
    void computeCurrentModel(int nVoteThresh = 1);
    bool isPointTooLarge(const dlovi::Matrix & matPoint) const;
    std::vector<int> & filterOutGiantPoints(std::vector<int> & arrPointIndices) const;
    std::vector<std::vector<int> > & filterOutGiantPointsFromCurrentVisList(std::vector<std::vector<int> > & arrVisLists) const;
    double timestamp() const;

    // Member Variables
    dlovi::compvis::SFMTranscript * m_pTranscript;
    dlovi::FreespaceDelaunayAlgorithm * m_pAlgorithm;
    dlovi::FreespaceDelaunayAlgorithm::Delaunay3 m_objDelaunay;
    std::vector<dlovi::FreespaceDelaunayAlgorithm::Delaunay3::Vertex_handle> m_arrVertexHandles;
    std::vector<dlovi::Matrix> m_arrModelPoints;
    std::list<dlovi::Matrix> m_lstModelTris;
    int m_nCurrentEntryIndex;
    std::set<int> m_setGiantPoints;
};

#endif

