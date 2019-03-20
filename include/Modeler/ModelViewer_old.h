// -*- c++ -*-
// Copyright 2009 David Lovi
//
// ModelViewer.h
//
// Defines the ModelViewer class
//
// This defines a simple model viewer widget, which can draw the 
// current model and the camera/keyframe poses / free-space constraints
// within it.
//
// TODO: Actually add drawing of FS constraints + a toggling option for this.
// 

#ifndef __MODEL_VIEWER_H
#define __MODEL_VIEWER_H

//#include "nappear/vdt.h"
//#include "nappear/fbo.h"

//#include "SurfaceInferer.h"
#include "MapMaker.h"
//#include <TooN/TooN.h>
//using namespace TooN;
//#include <TooN/se3.h>
#include <sstream>
#include "GLWindow2.h"

#include "OpenGL.h"
#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>
#include <cvd/colourspace_convert.h>
#include <cvd/colourspaces.h>
#include <cvd/utility.h>

#include "ATANCamera.h"

//#include "nappear/vdt.h"
//#include "nappear/fbo.h"

class ModelViewer{
public:
	// Constructors and Destructors
	//ModelViewer(SurfaceInferer & si, GLWindow2 & glw);
	ModelViewer(MapMaker & mm, GLWindow2 & glw, const ATANCamera &cam);

	// Getters:
	bool isModelUpdateRequested() const { return m_bModelUpdateRequested; }
	bool isModelUpdateDone() const {return m_bModelUpdateDone; }

	// Public Methods:
	void DrawModel(CVD::Image<CVD::Rgb<CVD::byte> > & imFrame, SE3<> se3CamFromWorld);
	std::string GetMessageForUser();
	void setUpdatedModel(const vector<dlovi::Matrix> & modelPoints, const list<dlovi::Matrix> & modelTris); // The SurfaceInferer's entry point for updating the model
	void markModelUpdateDone() { m_bModelUpdateDone = true; } // The SurfaceInferer's entry point for signaling the end of a model update
	void addKeyFrame(CVD::Image<CVD::Rgb<CVD::byte> > & im, SE3<> se3CamFromWorld){
		// OLD
		/*m_arrKeyFrames.push_back(std::make_pair(im, se3CamFromWorld));
        m_nNumKeyFrames++;*/
		// NEW
		CVD::Image<CVD::Rgb<CVD::byte> > im2(im.size());
		CVD::copy(im, im2);
		m_arrKeyFrames.push_back(std::make_pair(im2, se3CamFromWorld));
		m_nNumKeyFrames++;
	}
	void addKeyFrame(CVD::Image<CVD::byte> & im, SE3<> se3CamFromWorld){
		//std::cerr << "KFDALJF: " << im.size().x << " " << im.size().y << std::endl;
		CVD::Image<CVD::Rgb<CVD::byte> > im2(im.size());
		//std::cerr << "KFDALJF: " << im2.size().x << " " << im2.size().y << std::endl;
		CVD::convert_image(im, im2);
		m_arrKeyFrames.push_back(std::make_pair(im2, se3CamFromWorld));
		m_nNumKeyFrames++;
	}

protected:
	// Protected Methods
	void DrawPoints();
	void DrawTris();
	void DrawCamera(SE3<> se3, bool bSmall=false);
	void DrawGrid();
	void SetupFrustum();
	void SetupModelView(SE3<> se3WorldFromCurrent = SE3<>());
	void SetupLighting();
	void DisableLighting();
	void UpdateModel();
	void UpdateCenterOfMass();
	vector<dlovi::Matrix> & GetPoints();
	list<dlovi::Matrix> & GetTris();

	// Members
	ATANCamera mCamera;      // Same as the tracker's camera: N.B. not a reference variable!
	//SurfaceInferer & m_SurfaceInferer;
	MapMaker & m_MapMaker;
	GLWindow2 & mGLWindow;
	Vector<3> mv3MassCenter;
	SE3<> mse3ViewerFromWorld;
	std::ostringstream mMessageForUser;
	std::pair<vector<dlovi::Matrix>, list<dlovi::Matrix> > m_model;
	std::pair<vector<dlovi::Matrix>, list<dlovi::Matrix> > m_updatedModel;

	// Keyframes to texture with
	std::vector<std::pair<CVD::Image<CVD::Rgb<CVD::byte> >, SE3<> > > m_arrKeyFrames;
	int m_nNumKeyFrames;

	bool m_bModelUpdateRequested;
	bool m_bModelUpdateRunning;
	bool m_bModelUpdateDone;
	bool m_bDrawWire;
	bool m_bDrawPoints;
	bool m_bDrawGrid;
	bool m_bUse4Cams;
	ViewDependentTexture m_objVDT;
	nappear::FrameBufferObject * m_pFBO;

	static void drawMesh(int crap);
	static ModelViewer * pMe;
};

#endif
