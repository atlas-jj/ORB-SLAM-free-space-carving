#include "ModelViewer.h"
#include "KeyFrame.h"
#include <iomanip>

//#include <cvd/gl_helpers.h>

/*#include "OpenGL.h"
#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>*/

// HACK TO REPLACE:
//#include "nappear/glcommon.h"
//namespace nappear{
//GLenum printGLError_helper(const char * filename,int line);
//#define printGLError() printGLError_helper(__FILE__,__LINE__)
//};

//using namespace CVD;
using namespace std;
//using namespace nappear;

ModelViewer * ModelViewer::pMe = 0;
void ModelViewer::drawMesh(int crap){
	pMe->DrawTris();
}

// Constructors and Destructors

/*ModelViewer::ModelViewer(SurfaceInferer & si, GLWindow2 & glw)
	: m_SurfaceInferer(si), mGLWindow(glw) {
	mse3ViewerFromWorld = SE3<>::exp(makeVector(0,0,2,0,0,0)) * SE3<>::exp(makeVector(0,0,0,0.8 * M_PI,0,0));
  m_arrKeyFrames.reserve(500);
  m_nNumKeyFrames = 0;
	m_bModelUpdateRequested = false;
	m_bModelUpdateDone = true;
}*/

ModelViewer::ModelViewer(MapMaker & mm, GLWindow2 & glw, const ATANCamera &cam)
		: m_MapMaker(mm), mGLWindow(glw), mCamera(cam) {
	m_bDrawWire = true;
	m_bUse4Cams = true;
	m_bDrawGrid = true;
	m_bDrawPoints = false;
	//mse3ViewerFromWorld = SE3<>::exp(makeVector(0,0,2,0,0,0)) * SE3<>::exp(makeVector(0,0,0,0.8 * M_PI,0,0));
	mse3ViewerFromWorld = SE3<>::exp(makeVector(0,2,6,0,0,0)) * SE3<>::exp(makeVector(0,0,0,0.65 * M_PI,0,0));
	m_arrKeyFrames.reserve(500);
	m_nNumKeyFrames = 0;
	pMe = this;
	printGLError();
	m_objVDT.initialize("shaders/multi_tex.vsh", "shaders/multi_tex_ptam_dist2.fsh", 4);
	//m_objVDT.initialize("shaders/multi_tex.vsh", "shaders/multi_tex_current.fsh", 4);
	m_pFBO = new nappear::FrameBufferObject(1024, 1024);
	printGLError();

	m_bModelUpdateRequested = false;
	m_bModelUpdateDone = true;
}

// Public Methods

void ModelViewer::DrawModel(Image<Rgb<byte> > & imFrameOld, SE3<> se3CamFromWorldOld)
{
	printGLError();

	int nLastKF = m_nNumKeyFrames - 1;

	// Select the 4 KFs:

	Image<Rgb<byte> > imFrame[4];
	SE3<> se3CamFromWorld[4];
	if(m_bUse4Cams){
		// 4 most recent KFs
		imFrame[0] = m_arrKeyFrames[std::max(0, nLastKF)].first;
		imFrame[1] = m_arrKeyFrames[std::max(0, nLastKF-1)].first;
		imFrame[2] = m_arrKeyFrames[std::max(0, nLastKF-2)].first;
		imFrame[3] = m_arrKeyFrames[std::max(0, nLastKF-3)].first;
		se3CamFromWorld[0] = m_arrKeyFrames[std::max(0, nLastKF)].second;
		se3CamFromWorld[1] = m_arrKeyFrames[std::max(0, nLastKF-1)].second;
		se3CamFromWorld[2] = m_arrKeyFrames[std::max(0, nLastKF-2)].second;
		se3CamFromWorld[3] = m_arrKeyFrames[std::max(0, nLastKF-3)].second;
	}
	else{
		// 1 most recent KF
		imFrame[0] = m_arrKeyFrames[std::max(0, nLastKF)].first;
		imFrame[1] = m_arrKeyFrames[std::max(0, nLastKF)].first;
		imFrame[2] = m_arrKeyFrames[std::max(0, nLastKF)].first;
		imFrame[3] = m_arrKeyFrames[std::max(0, nLastKF)].first;
		se3CamFromWorld[0] = m_arrKeyFrames[std::max(0, nLastKF)].second;
		se3CamFromWorld[1] = m_arrKeyFrames[std::max(0, nLastKF)].second;
		se3CamFromWorld[2] = m_arrKeyFrames[std::max(0, nLastKF)].second;
		se3CamFromWorld[3] = m_arrKeyFrames[std::max(0, nLastKF)].second;
	}

	// Upload the image to our frame texture
	CVD::ImageRef ir = imFrame[0].size();

	glEnable(GL_TEXTURE_2D);
	static unsigned int frametex[4] = {0, 0, 0, 0};
	if(!frametex[0])
		glGenTextures(4, frametex);

	for(int i = 0; i < 4; i++){
		glBindTexture(GL_TEXTURE_2D, frametex[i]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
					 ir.x, ir.y, 0,
					 GL_RGB,
					 GL_UNSIGNED_BYTE,
					 imFrame[i].data());
	}

	printGLError();

	mMessageForUser.str(""); // Wipe the user message clean

	// Update viewer position according to mouse input:
	{
		pair<Vector<6>, Vector<6> > pv6 = mGLWindow.GetMousePoseUpdate();
		SE3<> se3CamFromMC;
		se3CamFromMC.get_translation() = mse3ViewerFromWorld * mv3MassCenter;
		mse3ViewerFromWorld = SE3<>::exp(pv6.first) *
							  se3CamFromMC * SE3<>::exp(pv6.second) * se3CamFromMC.inverse() * mse3ViewerFromWorld;
	}

	// Refresh the model to the most recent one
	UpdateModel();
	UpdateCenterOfMass();

	// Set up OpenGL stuff
	mGLWindow.SetupViewport();
	glClearColor(0,0,0,0);
	glClearDepth(1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColorMask(1,1,1,1);

	glEnable(GL_DEPTH_TEST);

	// Draw the grid
	if(m_bDrawGrid){
		glDisable(GL_TEXTURE_2D);
		DrawGrid();
		glEnable(GL_TEXTURE_2D);
	}

	// Draw the model
	SetupFrustum();
	SetupModelView();

	//vector<pair<double,int > > dists;
	double mod[16];
	glGetDoublev(GL_MODELVIEW_MATRIX,mod);


/*  Vec3d  curz(-mod[2],-mod[6],-mod[10]);
  for(int i=0;i<images.size();i++){
    double sc=1.0-Vec3d(Es[i].data[8],Es[i].data[9],Es[i].data[10]).dot(curz);
    dists.push_back(pair<double,int>(sc,i));
  }
  std::sort(dists.begin(),dists.end());
*/
	if(1){
		printGLError();
		for(int i=0; i<4; i++){

			int index;
			if(m_bUse4Cams){
				// Using Last 4 KFs:
				index = std::max(0, nLastKF-i);
			}
			else{
				// Using just the last KF:
				index = std::max(0, nLastKF);
			}

			// Extract & massage CLB Data from the Keyframes & from the ATANCamera
			dlovi::Matrix K(3, 3, 0.0); // 3x3 intrinsics
			dlovi::Matrix E(4, 4, 0.0); // 4x4 extrinsics [R, t; 0 0 0 1]

			for(int j = 0; j < 3; j++){
				for(int k = 0; k < 3; k++)
					E(j, k) = m_arrKeyFrames[index].second.get_rotation().get_matrix()(j, k);
				E(j, 3) = m_arrKeyFrames[index].second.get_translation()[j];
			}
			E(3, 3) = 1.0;

			// H&Z Version w/ positive fx, fy:
			K(0, 0) = mCamera.getFocalX(); // Negate for image center = topleft, y increasing downward (not H&Z)
			K(0, 2) = mCamera.getImageCenterX();
			K(1, 1) = mCamera.getFocalY(); // Negate for image center = topleft, y increasing downward (not H&Z)
			K(1, 2) = mCamera.getImageCenterY();
			K(2, 2) = 1.0;

			E = E.transpose();
			K = K.transpose();

			double cc[3] = {m_arrKeyFrames[index].second.inverse().get_translation()[0],
							m_arrKeyFrames[index].second.inverse().get_translation()[1],
							m_arrKeyFrames[index].second.inverse().get_translation()[2]};

			//int cam=dists[i].second;
			m_objVDT.setCamera(i, ir.x, ir.y,
							   K, E, cc,
							   0.01f, 400.f,frametex[i]);//,depthTextures[cam]);
		}
		//m_objVDT.updateDepthTextures(m_pFBO, drawMesh);
		boost::function<void (int)> func = drawMesh;
		m_objVDT.updateDepthTextures(m_pFBO, func);

		m_objVDT.enableProjectiveTexture();

		//if (g_usingPtamDist) {
		//float g_omega = .93f; // a guess
		float g_omega = .955416f; // from camera.cfg
		float omegas[4] = {g_omega, g_omega, g_omega, g_omega};
		float tan_omega2s[4] = {tan(g_omega/2), tan(g_omega/2), tan(g_omega/2), tan(g_omega/2)};
		m_objVDT.setShaderParameters1fv("omegas", omegas, 4);
		m_objVDT.setShaderParameters1fv("tan_omega2s", tan_omega2s, 4);
		//}

		DrawTris();
		m_objVDT.disableProjectiveTexture();
	}


	// Draw the wire-frame
	if(m_bDrawWire){
		glPushAttrib(GL_ENABLE_BIT);
		glDisable(GL_LIGHTING);
		glDisable(GL_TEXTURE_2D);
		//glColor3f(1, 1, 1);
		glColor4f(1, 1, 1, 0.2);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		SetupFrustum();
		SetupModelView();
		DrawTris();
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glPopAttrib();
	}

	glDisable(GL_TEXTURE_2D);
	if(m_bDrawPoints){
		DrawPoints();
	}

	// Draw the cameras
	DrawCamera(se3CamFromWorld[0]); // TODO: mark
	/*for(size_t i = 0; i < (unsigned)m_SurfaceInferer.numCams(); i++){
          dlovi::Matrix cc = m_SurfaceInferer.getCamCenter(i);
          SE3<> se3Tmp;
          se3Tmp.get_translation()[0] = -cc(0);
          se3Tmp.get_translation()[1] = -cc(1);
          se3Tmp.get_translation()[2] = -cc(2);
          DrawCamera(se3Tmp, true);
    }*/
	glDisable(GL_DEPTH_TEST);
	//glMatrixMode(GL_TEXTURE);
	//glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	mMessageForUser << " Model: " << GetPoints().size() << "P, " << GetTris().size() << "T";
	mMessageForUser << setprecision(4);
	mMessageForUser << "   Camera Pos: " << se3CamFromWorld[0].inverse().get_translation();

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-1.f, 1.f, -1, 1, -1, 1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(.5f, -1.f, 0.f);
	glScalef(.3, .3, 1.f);

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, frametex[0]);
	glBegin(GL_QUADS);
	glTexCoord2f(0.f, 1.f);
	glVertex2f(0.f, 0.f);
	glTexCoord2f(1.f, 1.f);
	glVertex2f(1.f, 0.f);
	glTexCoord2f(1.f, 0.f);
	glVertex2f(1.f, 1.f);
	glTexCoord2f(0.f, 0.f);
	glVertex2f(0.f, 1.f);
	glEnd();
}

string ModelViewer::GetMessageForUser(){
	return mMessageForUser.str();
}

void ModelViewer::setUpdatedModel(const vector<dlovi::Matrix> & modelPoints, const list<dlovi::Matrix> & modelTris){
	m_updatedModel.first = modelPoints; m_updatedModel.second = modelTris;
}

// Protected Methods

void ModelViewer::DrawPoints(){
	SetupFrustum();
	SetupModelView();

	glColor3f(0,1,1);
	glPointSize(3);
	glBegin(GL_POINTS);
	for(size_t i = 0; i < GetPoints().size(); i++)
		glVertex3d(GetPoints()[i](0), GetPoints()[i](1), GetPoints()[i](2));
	glEnd();
}

void ModelViewer::DrawTris(){
	//SetupFrustum();
	//SetupModelView();

	// Triangle Color
	//glColor4f(0.7f, 0.7f, 0.9f, 0.7f);
	glBegin(GL_TRIANGLES);
	for(list<dlovi::Matrix>::const_iterator it = GetTris().begin(); it != GetTris().end(); it++){
		glTexCoord3d(GetPoints()[(*it)(0)](0), GetPoints()[(*it)(0)](1), GetPoints()[(*it)(0)](2));
		glVertex3d(GetPoints()[(*it)(0)](0), GetPoints()[(*it)(0)](1), GetPoints()[(*it)(0)](2));

		glTexCoord3d(GetPoints()[(*it)(1)](0), GetPoints()[(*it)(1)](1), GetPoints()[(*it)(1)](2));
		glVertex3d(GetPoints()[(*it)(1)](0), GetPoints()[(*it)(1)](1), GetPoints()[(*it)(1)](2));

		glTexCoord3d(GetPoints()[(*it)(2)](0), GetPoints()[(*it)(2)](1), GetPoints()[(*it)(2)](2));
		glVertex3d(GetPoints()[(*it)(2)](0), GetPoints()[(*it)(2)](1), GetPoints()[(*it)(2)](2));
	}
	glEnd();
}

void ModelViewer::DrawCamera(SE3<> se3CfromW, bool bSmall){
	SetupModelView(se3CfromW.inverse());
	SetupFrustum();

	if(bSmall)
		glLineWidth(1);
	else
		glLineWidth(3);

	glBegin(GL_LINES);
	glColor3f(1,0,0);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.1f, 0.0f, 0.0f);
	glColor3f(0,1,0);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.1f, 0.0f);
	glColor3f(1,1,1);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.1f);
	glEnd();

	if(!bSmall){
		glLineWidth(1);
		glColor3f(0.5,0.5,0.5);
		SetupModelView();
		Vector<2> v2CamPosXY = se3CfromW.inverse().get_translation().slice<0,2>();
		glBegin(GL_LINES);
		glColor3f(1,1,1);
		glVertex2d(v2CamPosXY[0] - 0.04, v2CamPosXY[1] + 0.04);
		glVertex2d(v2CamPosXY[0] + 0.04, v2CamPosXY[1] - 0.04);
		glVertex2d(v2CamPosXY[0] - 0.04, v2CamPosXY[1] - 0.04);
		glVertex2d(v2CamPosXY[0] + 0.04, v2CamPosXY[1] + 0.04);
		glEnd();
	}
}

void ModelViewer::DrawGrid(){
	SetupFrustum();
	SetupModelView();
	glLineWidth(1);

	glBegin(GL_LINES);

	// Draw a larger grid around the outside..
	double dGridInterval = 0.1;

	double dMin = -100.0 * dGridInterval;
	double dMax =  100.0 * dGridInterval;

	for(int x=-10;x<=10;x+=1){
		if(x==0)
			glColor3f(1,1,1);
		else
			glColor3f(0.3,0.3,0.3);
		glVertex3d((double)x * 10 * dGridInterval, dMin, 0.0);
		glVertex3d((double)x * 10 * dGridInterval, dMax, 0.0);
	}
	for(int y=-10;y<=10;y+=1){
		if(y==0)
			glColor3f(1,1,1);
		else
			glColor3f(0.3,0.3,0.3);
		glVertex3d(dMin, (double)y * 10 *  dGridInterval, 0.0);
		glVertex3d(dMax, (double)y * 10 * dGridInterval, 0.0);
	}

	glEnd();

	glBegin(GL_LINES);
	dMin = -10.0 * dGridInterval;
	dMax =  10.0 * dGridInterval;

	for(int x=-10;x<=10;x++){
		if(x==0)
			glColor3f(1,1,1);
		else
			glColor3f(0.5,0.5,0.5);

		glVertex3d((double)x * dGridInterval, dMin, 0.0);
		glVertex3d((double)x * dGridInterval, dMax, 0.0);
	}
	for(int y=-10;y<=10;y++){
		if(y==0)
			glColor3f(1,1,1);
		else
			glColor3f(0.5,0.5,0.5);
		glVertex3d(dMin, (double)y * dGridInterval, 0.0);
		glVertex3d(dMax, (double)y * dGridInterval, 0.0);
	}

	glColor3f(1,0,0);
	glVertex3d(0,0,0);
	glVertex3d(1,0,0);
	glColor3f(0,1,0);
	glVertex3d(0,0,0);
	glVertex3d(0,1,0);
	glColor3f(1,1,1);
	glVertex3d(0,0,0);
	glVertex3d(0,0,1);
	glEnd();

	//   glColor3f(0.8,0.8,0.8);
	//   glRasterPos3f(1.1,0,0);
	//   mGLWindow.PrintString("x");
	//   glRasterPos3f(0,1.1,0);
	//   mGLWindow.PrintString("y");
	//   glRasterPos3f(0,0,1.1);
	//   mGLWindow.PrintString("z");
}

void ModelViewer::SetupFrustum(){
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	double zNear = 0.03;
	glFrustum(-zNear, zNear, 0.75*zNear,-0.75*zNear,zNear,50);
	glScalef(1,1,-1);
	return;
}

void ModelViewer::SetupModelView(SE3<> se3WorldFromCurrent){
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMultMatrix(mse3ViewerFromWorld * se3WorldFromCurrent);
	return;
}

void ModelViewer::SetupLighting(){
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glShadeModel(GL_FLAT);
}

void ModelViewer::DisableLighting(){
	glDisable(GL_LIGHTING);
}

void ModelViewer::UpdateModel(){
	//m_model = m_SurfaceInferer.getModel(); // crashes here

	if(m_bModelUpdateRequested && ! m_bModelUpdateDone)
		return;

	if(m_bModelUpdateRequested && m_bModelUpdateDone){
		m_model = m_updatedModel;
		m_bModelUpdateRequested = false;
		return;
	}

	m_bModelUpdateDone = false;
	m_bModelUpdateRequested = true; // implicitly signals SurfaceInferer thread which is polling
}

void ModelViewer::UpdateCenterOfMass(){
	int nForMass = 0;
	mv3MassCenter = Zeros;

	for(size_t i = 0; i < GetPoints().size(); i++){
		dlovi::Matrix & v3Pos = GetPoints()[i];

		if(v3Pos.dot(v3Pos) < 10000){
			nForMass++;
			mv3MassCenter[0] += v3Pos(0);
			mv3MassCenter[1] += v3Pos(1);
			mv3MassCenter[2] += v3Pos(2);
		}
	}
	mv3MassCenter = mv3MassCenter / (0.1 + nForMass);
}

vector<dlovi::Matrix> & ModelViewer::GetPoints(){
	return m_model.first;
}

list<dlovi::Matrix> & ModelViewer::GetTris(){
	return m_model.second;
}

