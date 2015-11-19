
#define USE_OPENCV

#define _CRT_SECURE_NO_WARNINGS
#include <algorithm>
#include <string>
#include <complex>
#include <vector>

#ifdef USE_OPENCV
#pragma warning(disable: 4819)
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif //USE_OPENCV

#include "gl/freeglut.h"

#pragma comment(linker,"/SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup")

using namespace std;

enum LightState {
	CW,
	Pulse,
	Off
};
enum VectorState {
	Arrow,
	Point,
	NoVec
};
const double pi = acos(0.0) * 2;

/* initialize */
bool camcont = false;
double currentTime = 0;
int lastMx = 0;
int lastMy = 0;
bool firstMouse = true;
string command = "";
bool stop = false;
double t0 = -10000;
double delta = 1e-10;


LightState ls = CW; // 光の状態
VectorState vs = Point; // ベクトルの表記
bool projectionFlag = false; // 射影の表示
bool stringFlag = false; // 文字列の表示
bool frontFlag = true; // 最初の偏光板の表示


#ifdef USE_OPENCV
bool recordFlag = false;
const unsigned int cc = 3;
void* dataBuffer = NULL;
IplImage *frame;
CvVideoWriter* vw;
int recordCount = 0;
double recordTime = 1000.0; // 録画時間
double videoFps = 29.7;  // 動画のfps
#endif //USE_OPENCV


// window
int width = 800;
int height = 600; 
int fps = 60;
string windowTitle = "Polarization";

// camera position
double camtheta = -pi / 4; // rad
double camphi = pi*7 / 16; // rad
double camdist = 100000; // (nm) 0,0,0からのカメラの距離
double camSpeed = 0.01; // (rad/pix)
double camscaling = 1.05; // 拡大縮小時のスケール
double camFar = 3000000; // (nm)
double camNear = 100; // (nm)
double camFov = 5.0; // (degree)
double camsmooththeta = camtheta;
double camsmoothphi = camphi;
double camsmoothdist = camdist;
double camMinSpeed = pi / 1e4;
double camratio = 0.05; // カメラ移動時の追従 1で一瞬で移動

// anime speed
double fastAnimeSpeed = 1e-4;
double normalAnimeSpeed = 3e-6;
double slowAnimeSpeed = 1e-6;
double animeSpeed = normalAnimeSpeed; // (ps per realSec) 1e-12

// physical value
double lightSpeed = 3e5; //(m/ms) 1e-3
double waveLength = 780; // (nm) 1e-9
double lightFreq = lightSpeed/waveLength; // (THz) 1e12

// mesh ground
double meshInterval = 200; //(nm)　メッシュのマス
int meshCountZ = (int)(5*waveLength/meshInterval); // num 進行方向のメッシュの数
int meshCountX = 10; // num 電場方向のメッシュの数
double meshZShift = -waveLength*1.5; // (nm) 表示するメッシュの原点からのオフセット

// arrows
double arrowInterval = waveLength / 30; //(nm) ベクトル表示の間隔
int arrowCount = (int)(meshCountZ * 2 * meshInterval / arrowInterval); // num 表示数
double arrowHeight = waveLength /1; // (nm) 矢の円柱の高さ
double arrowRadius = arrowInterval / 6; // (nm) 矢の円柱の半径
double coneHeight = arrowRadius * 2; // (nm) 矢の先端の高さ
double coneRadius = arrowRadius * 4; // (nm) 矢の先端の半径
double sphereRadius = coneRadius * 2; // (nm) 球体表示時の球体の半径
bool ghostArrowFlag = false; // 波長版不使用時を別に表示するフラグ

// sheet
int sheetSlice = 100; // num スクリーンの分割数
double sheetSize = arrowHeight*1.5; // (nm) スクリーンや波長版などの大きさ
double spotSize = 6.0; // スクリーンのスポットの大きさ
double framelength = 30.0; // 波長/偏光板などの白枠の大きさ
double frameoffset = 5.0; // 波長/偏光板などの白枠と本体との間隔

// polarizer
bool polarizerFlag0 = true;
bool polarizerFlag1 = false;
bool polarizerFlag2 = false;
bool polarizerRotateFlag0 = false;
bool polarizerRotateFlag1 = false;
bool polarizerRotateFlag2 = false;
double phi = 0; // input phase shift
double theta = pi / 2; // input polarization
double polarizerTheta1 = 0;
double polarizerTheta2 = 0;
double polarizerRotateSpeed = 0.005; // (rad) 波長板や偏光板の回転時の測度
double polarizerPos1 = 4 * meshCountZ*meshInterval / 3; // (nm) 手前偏光板の位置
double polarizerPos2 = 2 * meshCountZ*meshInterval / 3; // (nm) 奥の偏光板の位置

// waveplate
bool wavePlateFlag = false;
bool wavePlateRotateFlag = false;
double wavePlateTheta = 0;
double wavePlateLength = waveLength*2; //(nm)　偏光板で位相をwavePlatePhaseだけずらすのにかかる距離
double wavePlatePhase = pi * 2; //(rad)

// screen object
unsigned int historyCount=30; // スクリーンに映す点の履歴の数
vector<pair<double, double > > history;

// decay speed
double pulseWidth = 4*waveLength/lightSpeed; // (ps) gaussian sigma パルス出力時のパルス時間幅
double pulsePower = 1e0; // (nm) gaussian amplitude パルスの強度
double decaySpeed = 1e-3; // (ps) speed of switch　光をOffにしたりCWにした時の減衰時間
double polarizerDecayLength = meshInterval/2; // (nm) 偏光板通過時の減衰の距離スケール


void drawString(string);
void drawGround();
void drawArrow(double);
void drawBothArrow(bool);
void drawAlphaArrow(double, double);
void drawSphere(double, bool, double);
void drawAlphaSphere(double, double);
void drawSheet(double);
void drawPlate(double, double);
void drawMeshedSheet(double);
void procCommand();
void drawCircle();
void drawAlphaCircle(double);

double gaussian(double t) {
	return pulsePower * exp(-pow(t - (t0+pulseWidth*3),2) / 2 / pow(pulseWidth,2));
}
double expdecay(double t) {
	return exp(-max(0.0,(t - t0))/decaySpeed);
}
double expup(double t) {
	return 1-exp(-max(0.0, (t - t0)) / decaySpeed);
}

void setElectricField(double pos, double* vx, double* vy,double* amplitude) {

	// initialize
	double lightPhase = (pos / waveLength - currentTime*lightFreq) * 2 * pi;
	complex<double> i = complex<double>(0, 1);
	complex<double> px = cos(theta)*exp(i*lightPhase);
	complex<double> py = sin(theta)*exp(i*(lightPhase+phi));
	complex<double> ppx, ppy;

	// polarizer 2
	if (polarizerFlag2) {
		ppx = px*cos(polarizerTheta2) + py*sin(polarizerTheta2);
		ppy = -px*sin(polarizerTheta2) + py*cos(polarizerTheta2);
		ppy *= 2.0 / (exp(max(0.0, pos - polarizerPos2) / polarizerDecayLength) + 1);
		px = ppx*cos(polarizerTheta2) - ppy * sin(polarizerTheta2);
		py = ppx*sin(polarizerTheta2) + ppy * cos(polarizerTheta2);
	}

	// waveplate
	if (wavePlateFlag) {
		ppx = px*cos(wavePlateTheta) + py*sin(wavePlateTheta);
		ppy = -px*sin(wavePlateTheta) + py*cos(wavePlateTheta);
		ppx *= exp(i*min(max(0.0,pos-polarizerPos2),wavePlateLength)/wavePlateLength*wavePlatePhase);
		px = ppx*cos(wavePlateTheta) - ppy * sin(wavePlateTheta);
		py = ppx*sin(wavePlateTheta) + ppy * cos(wavePlateTheta);
	}

	// polarizer 1
	if (polarizerFlag1) {
		ppx = px*cos(polarizerTheta1) + py*sin(polarizerTheta1);
		ppy = -px*sin(polarizerTheta1) + py*cos(polarizerTheta1);
		ppy *= 2.0 / (exp(max(0.0, pos - polarizerPos1) / polarizerDecayLength) + 1);
		px = ppx*cos(polarizerTheta1) - ppy * sin(polarizerTheta1);
		py = ppx*sin(polarizerTheta1) + ppy * cos(polarizerTheta1);
	}


	// beam shape
	double decay=0;
	if (ls == Pulse) {
		decay = gaussian(currentTime - pos / lightSpeed);
	}
	if (ls == Off) {
		decay = expdecay(currentTime - pos / lightSpeed);
	}
	if (ls == CW) {
		decay = expup(currentTime - pos / lightSpeed);
	}
	px *= decay;
	py *= decay;

	// to real value
	(*vx) = real(px);
	(*vy) = real(py);
	(*amplitude) = sqrt(norm(px)+norm(py));
	return;
}

void display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadIdentity();

	// locate camera
	double cphi = min(max(camsmoothphi, delta), pi - delta);
	double ctheta = camsmooththeta;
	double eyex = camsmoothdist*cos(ctheta)*sin(cphi);
	double eyez = camsmoothdist*sin(ctheta)*sin(cphi);
	double eyey = camsmoothdist*cos(cphi);
	gluLookAt(eyex, eyey, eyez,0, 0, 0,0, 1, 0);

	// move camera
	if (fabs(camsmooththeta - camtheta) < camMinSpeed) {
		camsmooththeta = camtheta;
	} else {
		camsmooththeta = camratio*camtheta + (1-camratio)*camsmooththeta;
	}
	if (fabs(camsmoothphi - camphi) < camMinSpeed) {
		camsmoothphi = camphi;
	} else {
		camsmoothphi = camratio*camphi + (1 - camratio)*camsmoothphi;
	}
	if (fabs(camsmoothdist - camdist) < camMinSpeed) {
		camsmoothdist = camdist;
	} else {
		camsmoothdist = camratio*camdist + (1 - camratio)*camsmoothdist;
	}

	// draw string
	if (stringFlag) {
		drawString(command);
	}

	double px, py, amplitude;

	// ambient lighting
	GLfloat pos0[] = { 0, 1000, 0, 0.0 };
	glLightfv(GL_LIGHT0, GL_POSITION, pos0);
	GLfloat pos1[] = { 0, -1000, 0, 0.0 };
	glLightfv(GL_LIGHT1, GL_POSITION, pos1);
	GLfloat pos2[] = { 1000, 0, 0, 0.0 };
	glLightfv(GL_LIGHT2, GL_POSITION, pos2);

	// draw ground mesh
	drawGround();

	// draw screen history
	for (unsigned i = 0; i < history.size(); i++) {
		glPushMatrix();
		{
			px = history[i].first;
			py = history[i].second;
			double length = hypot(px,py);
			double rot = atan2(py, px);
			glTranslated(0, 0, arrowCount / 2*arrowInterval);
			glRotated(rot / 2 / pi * 360, 0, 0, 1);
			drawSphere(length, true, 1.0*i/history.size());
		}
		glPopMatrix();
	}

	// draw electric field
	for (int i = 0; i <= arrowCount; i++) {
		glPushMatrix();
		{
			setElectricField(i*arrowInterval, &px, &py, &amplitude);
			double length = hypot(px, py);
			double rot = atan2(py,px);
			glTranslated(0, 0, (i - arrowCount/2)*arrowInterval);
			glRotated(rot/2/pi*360, 0, 0, 1);
			if (vs == Arrow) {
				drawArrow(length);
			} else if(vs == Point) {
				drawSphere(length,(i==0 || i==arrowCount),1.0);
			}
			if (vs == NoVec && (i == 0 || i == arrowCount)) {
				drawSphere(length, (i == 0 || i == arrowCount), 1.0);
			}
			if (i == arrowCount) {
				history.push_back(make_pair(px,py));
				if (history.size()>historyCount) {
					history.erase(history.begin());
				}
			}
		}
		glPopMatrix();
	}

	// draw ghost electric field
	if (ghostArrowFlag && wavePlateFlag) {
		wavePlateFlag = false;
		glEnable(GL_BLEND);
		for (int i = 0; i <= arrowCount; i++) {
			if (i*arrowInterval <= polarizerPos2)continue;

			glPushMatrix();
			{
				setElectricField(i*arrowInterval, &px, &py, &amplitude);
				double length = hypot(px, py);
				double rot = atan2(py, px);
				glTranslated(0, 0, (i - arrowCount / 2)*arrowInterval);
				glRotated(rot / 2 / pi * 360, 0, 0, 1);

				if (vs==Arrow) {
					drawAlphaArrow(length,1.0);
				} else if(vs==Point) {
					drawAlphaSphere(length, 1.0);
				}
			}
			glPopMatrix();
		}
		glDisable(GL_BLEND);
		wavePlateFlag = true;
	}

	// draw projection field
	if (projectionFlag) {
		for (int i = 0; i <= arrowCount; i++) {
			glPushMatrix();
			{
				setElectricField(i*arrowInterval, &px, &py, &amplitude);
				glPushMatrix();
				{
					glTranslated(meshZShift, py*arrowHeight, (i - arrowCount / 2)*arrowInterval);
					glRotated(90, 0, 0, 1);
					drawCircle();
				}
				glPopMatrix();
				glPushMatrix();
				{
					glTranslated(px*arrowHeight, meshZShift, (i - arrowCount / 2)*arrowInterval);
					drawCircle();
				}
				glPopMatrix();
			}
			glPopMatrix();

			if (wavePlateFlag && ghostArrowFlag) {
				glEnable(GL_BLEND);
				wavePlateFlag = false;
				glPushMatrix();
				{
					setElectricField(i*arrowInterval, &px, &py, &amplitude);
					glPushMatrix();
					{
						glTranslated(meshZShift, py*arrowHeight, (i - arrowCount / 2)*arrowInterval);
						glRotated(90, 0, 0, 1);
						drawAlphaCircle(0.2);
					}
					glPopMatrix();
					glPushMatrix();
					{
						glTranslated(px*arrowHeight, meshZShift, (i - arrowCount / 2)*arrowInterval);
						drawAlphaCircle(0.2);
					}
					glPopMatrix();
				}
				glPopMatrix();
				wavePlateFlag = true;
				glDisable(GL_BLEND);
			}

		}
	}

	// draw front sheet
	if (frontFlag) {
		glPushMatrix();
		{
			glTranslated(0, 0, -arrowCount / 2 * arrowInterval);
			glRotated(theta / 2 / pi * 360 + 180, 0, 0, 1);
			drawBothArrow(false);
			glPushMatrix();
			{
				drawSheet(sheetSize);
			}
			glPopMatrix();
		}
		glPopMatrix();
	}

	// draw screen
	drawMeshedSheet(sheetSize);

	// draw wave plate
	if (wavePlateFlag) {
		glEnable(GL_BLEND);
		glPushMatrix();
		{
			glRotated(wavePlateTheta / 2 / pi * 360 + 180, 0, 0, 1);
			glTranslated(0, 0, polarizerPos2 - arrowCount * arrowInterval / 2);
			drawPlate(sheetSize,wavePlateLength);
			drawBothArrow(true);
		}
		glPopMatrix();
		glDisable(GL_BLEND);
	}

	// draw polarizer1
	if (polarizerFlag1) {
		glEnable(GL_BLEND);
		glPushMatrix();
		{
			glRotated(polarizerTheta1 / 2 / pi * 360 + 180, 0, 0, 1);
			glTranslated(0, 0, polarizerPos1 -  arrowCount* arrowInterval/2);
			drawSheet(sheetSize);
			drawBothArrow(false);
		}
		glPopMatrix();
		glDisable(GL_BLEND);
	}

	// draw polarizer2
	if (polarizerFlag2) {
		glEnable(GL_BLEND);
		glPushMatrix();
		{
			glRotated(polarizerTheta2 / 2 / pi * 360 + 180, 0, 0, 1);
			glTranslated(0, 0, polarizerPos2 - arrowCount* arrowInterval / 2);
			drawSheet(sheetSize);
			drawBothArrow(false);
		}
		glPopMatrix();
		glDisable(GL_BLEND);
	}
	glFlush();
	glutSwapBuffers();

#ifdef USE_OPENCV
	// add frame to video when record
	if (recordFlag) {
		frame = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, cc);
		glReadBuffer(GL_BACK);
		glReadPixels(0, 0, width, height, GL_BGR_EXT, GL_UNSIGNED_BYTE, dataBuffer);
		GLubyte* p = static_cast<GLubyte*>(dataBuffer);
		for (int j = 0; j < height; ++j) {
			for (int i = 0; i < width; ++i) {
				frame->imageData[(height - j - 1) * frame->widthStep + i * 3 + 0] = *p;
				frame->imageData[(height - j - 1) * frame->widthStep + i * 3 + 1] = *(p + 1);
				frame->imageData[(height - j - 1) * frame->widthStep + i * 3 + 2] = *(p + 2);
				p += cc;
			}
		}
		cvWriteFrame(vw, frame);
		cvReleaseImage(&frame);
		recordCount ++;

		if (1.0*recordCount/fps > recordTime) {
			cvReleaseVideoWriter(&vw);
			vw = NULL;
			free(dataBuffer);
			recordFlag = false;
		}
	}
#endif //USE_OPENCV
}

void timer(int value) {
	glutTimerFunc(1000/fps, timer, 0);
	if (!stop) {
		currentTime += 1000.0 / fps * animeSpeed;
		if (polarizerRotateFlag0) theta += polarizerRotateSpeed;
		if (polarizerRotateFlag1) polarizerTheta1 += polarizerRotateSpeed;
		if (polarizerRotateFlag2) polarizerTheta2 += polarizerRotateSpeed;
		if (wavePlateRotateFlag) wavePlateTheta += polarizerRotateSpeed;
	}
	glutPostRedisplay();
}

void keyboard(unsigned char key, int mx, int my) {
	if (key == VK_ESCAPE) {
		glutLeaveMainLoop();
	}
	if (key == VK_SPACE) {
		glutFullScreenToggle();
	}

	// input command
	if ('a' <= key && key <= 'z' || key == '=' || '0' <= key && key <= '9') {
		command += key;
	}
	if (key == VK_BACK) {
		if (command.length() > 0) {
			command = command.substr(0,command.length()-1);
		}
	}
	if (key == VK_RETURN) {
		procCommand();
		command = "";
	}

	// camera rotation
	if (key == 'W') {
		camphi -= pi/32;
		if (camphi < 0)camphi = 0;
	}
	if (key == 'S') {
		camphi += pi/32;
		if (camphi > pi)camphi = pi;
	}
	if (key == 'D') {
		camtheta += pi/16;
	}
	if (key == 'A') {
		camtheta -= pi/16;
	}

	// camera zooming
	if (key == 'F') {
		camdist *= 1.05;
	}
	if (key == 'R') {
		camdist /= 1.05;
	}

	// move camera to registered position
	if (key == 'I') {
		camtheta = 0;
		camphi = 0;
	}
	if (key == 'K') {
		camtheta = 0;
		camphi = pi/2;
	}
	if (key == 'J') {
		camtheta = pi / 2;
		camphi = pi / 2;
	}
	if (key == 'L') {
		camtheta = -pi/2;
		camphi = pi/2;
	}
	if (key == 'U') {
		camtheta = pi *3/ 16;
		camphi = pi *6/ 16;
	}
	if (key == 'O') {
		camtheta = -pi *3/ 16;
		camphi = pi *6/ 16;
	}
}

void reshape(int w, int h) {
	width = w;
	height = h;
	// reset camera view
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(camFov, 1.0*width / height, camNear, camFar);
	glMatrixMode(GL_MODELVIEW);
}

// camera scaling
void mouseWheel(int wheelNumber,int direction,int x,int y) {
	if (direction == 1) {
		camdist *= camscaling;
	} else {
		camdist /= camscaling;
	}
}
// camera lock
void mouse(int button, int state, int x, int y) {
	if (button == GLUT_LEFT_BUTTON) {
		if (state == GLUT_UP) {
			camcont = false;
		} else if(state==GLUT_DOWN){			
			camcont = true;
			lastMx = x;
			lastMy = y;
		}
	}
}
// camera move
void motion(int x, int y) {
	if (camcont) {
		int difx = x - lastMx;
		int dify = y - lastMy;
		camtheta += difx*camSpeed;
		camphi -= dify*camSpeed;
		lastMx = x;
		lastMy = y;
		if (camphi < 0) camphi = 0;
		if (camphi > pi) camphi = pi;
	}
}


int main(int argc,char** argv) {
	glutInit(&argc, argv);;
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH);
	glutInitWindowSize(width, height);
	glutCreateWindow(windowTitle.c_str());

	// callback
	glutDisplayFunc(display);
	glutTimerFunc(1000/fps,timer,0);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutMouseWheelFunc(mouseWheel);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);

	// alpha setting
	glClearColor(0, 0, 0, 1);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glFrontFace(GL_CW);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);
	glEnable(GL_AUTO_NORMAL);
	glShadeModel(GL_SMOOTH);

	// lighting
	glEnable(GL_LIGHTING);
	{
		glEnable(GL_LIGHT0);
		GLfloat lightpos[] = { 0.0f, 1000.0f,0.0f, 0.0f };
		GLfloat lightdif[] = { 0.6f, 0.6f, 0.6f, 1.0f };
		GLfloat lightamb[] = { 0.4f, 0.4f, 0.4f, 1.0f };
		GLfloat lightspe[] = { 1.0f, 1.0f, 1.0f, 1.0f };
		glLightfv(GL_LIGHT0, GL_POSITION, lightpos);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, lightdif);
		glLightfv(GL_LIGHT0, GL_SPECULAR, lightspe);
		glLightfv(GL_LIGHT0, GL_AMBIENT, lightamb);
	}
	{
		glEnable(GL_LIGHT1);
		GLfloat lightpos[] = { 0.0, -1000.0, 0.0, 0.0 };
		GLfloat lightdif[] = { 0.5f, 0.5f, 0.5f, 1.0f };
		GLfloat lightamb[] = { 0.3f, 0.3f, 0.3f, 1.0f };
		GLfloat lightspe[] = { 1.0f, 1.0f, 1.0f, 1.0f };
		glLightfv(GL_LIGHT1, GL_POSITION, lightpos);
		glLightfv(GL_LIGHT1, GL_DIFFUSE, lightdif);
		glLightfv(GL_LIGHT1, GL_SPECULAR, lightspe);
		glLightfv(GL_LIGHT1, GL_AMBIENT, lightamb);
	}
	{
		glEnable(GL_LIGHT1);
		GLfloat lightpos[] = { 1000.0, 0.0, 0.0, 0.0 };
		GLfloat lightdif[] = { 0.5f, 0.5f, 0.5f, 1.0f };
		GLfloat lightamb[] = { 0.3f, 0.3f, 0.3f, 1.0f };
		GLfloat lightspe[] = { 1.0f, 1.0f, 1.0f, 1.0f };
		glLightfv(GL_LIGHT1, GL_POSITION, lightpos);
		glLightfv(GL_LIGHT1, GL_DIFFUSE, lightdif);
		glLightfv(GL_LIGHT1, GL_SPECULAR, lightspe);
		glLightfv(GL_LIGHT1, GL_AMBIENT, lightamb);
	}

	glutMainLoop();

#ifdef USE_OPENCV
	if (vw != NULL) {
		cvReleaseVideoWriter(&vw);
	}
#endif //USE_OPENCV

}



void drawString(string sstr) {
	const char* str = sstr.c_str();
	glPushMatrix();
	{
		glLoadIdentity();
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		{
			glDisable(GL_LIGHTING);
			glLoadIdentity();
			gluOrtho2D(0, glutGet(GLUT_WINDOW_WIDTH), 0, glutGet(GLUT_WINDOW_HEIGHT));
			glPushAttrib(GL_CURRENT_BIT | GL_DEPTH_BUFFER_BIT);
			glDisable(GL_DEPTH_TEST);
			glColor4d(1, 1, 1, 1);
			glRasterPos2d(20, 20);
			while (*str) {
				glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *str);
				str++;
			}
			glPopAttrib();
			glEnable(GL_LIGHTING);
		}
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
	}
	glPopMatrix();
}

void drawGround() {
	glEnable(GL_BLEND);
	// draw ground mesh
	GLfloat dif[] = { 0.4f, 0.4f, 0.4f, 0.2f };
	GLfloat amb[] = { 0.6f, 0.6f, 0.6f, 0.2f };
	GLfloat spe[] = { 0.0f, 0.0f, 0.0f, 0.2f };
	GLfloat emi[] = { 0.0f, 0.0f, 0.0f, 0.0f };
	float shi = 0;
	glPushMatrix();
	{
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, dif);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spe);
		glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emi);
		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shi);
		glBegin(GL_LINES);
		for (int p = -meshCountX+1; p < meshCountX; p++) {
			glVertex3d(p*meshInterval, meshZShift, -meshCountZ*meshInterval);
			glVertex3d(p*meshInterval, meshZShift, meshCountZ*meshInterval);
		}
		glEnd();
	}
	glPopMatrix();
	glPushMatrix();
	{
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, dif);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spe);
		glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emi);
		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shi);
		glBegin(GL_LINES);
		for (int p = -meshCountZ+1; p < meshCountZ; p++) {
			glVertex3d(-meshCountX*meshInterval, meshZShift, p*meshInterval);
			glVertex3d(meshCountX*meshInterval, meshZShift, p*meshInterval);
		}
		glEnd();
	}
	glPopMatrix();
	if (projectionFlag) {
		glPushMatrix();
		{
			glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, dif);
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
			glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spe);
			glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emi);
			glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shi);
			glBegin(GL_LINES);
			for (int p = -meshCountX+1; p < meshCountX; p++) {
				glVertex3d(meshZShift, p*meshInterval, -meshCountZ*meshInterval);
				glVertex3d(meshZShift, p*meshInterval, meshCountZ*meshInterval);
			}
			glEnd();
		}
		glPopMatrix();
		glPushMatrix();
		{
			glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, dif);
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
			glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spe);
			glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emi);
			glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shi);
			glBegin(GL_LINES);
			for (int p = -meshCountZ+1; p < meshCountZ; p++) {
				glVertex3d(meshZShift, meshCountX*meshInterval, p*meshInterval);
				glVertex3d(meshZShift, -meshCountX*meshInterval, p*meshInterval);
			}
			glEnd();
		}
		glPopMatrix();
	}
	glDisable(GL_BLEND);
}

void drawArrow(double amplitude) {
	glRotated(90.0, 0, 0, 1);
	glPushMatrix();
	{
		GLfloat dif[] = { 0.2f, 0.8f, 0.2f, 1.0f };
		GLfloat amb[] = { 0.2f, 0.2f, 0.2f, 1.0f };
		GLfloat spe[] = { 1.0f, 1.0f, 1.0f, 1.0f };
		GLfloat emi[] = { 0.0f, 0.0f, 0.0f, 0.0f };
		GLfloat shi = 50;
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, dif);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spe);
		glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emi);
		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shi);
		glRotated(90.0, 1, 0, 0);
		glutSolidCylinder(arrowRadius, arrowHeight * amplitude, 20, 20);
	}
	glPopMatrix();

	glPushMatrix();
	{
		GLfloat dif[] = { 0.2f, 0.9f, 0.2f, 1.0f };
		GLfloat amb[] = { 0.2f, 0.2f, 0.2f, 1.0f };
		GLfloat spe[] = { 1.0f, 1.0f, 1.0f, 1.0f };
		GLfloat emi[] = { 0.0f, 0.0f, 0.0f, 0.0f };
		GLfloat shi = 50;
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, dif);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spe);
		glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emi);
		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shi);
		glTranslated(0, -arrowHeight * amplitude, 0);
		glRotated(90.0, 1, 0, 0);
		glutSolidCone(coneRadius, coneHeight, 20, 20);
	}
	glPopMatrix();
}

void drawAlphaArrow(double amplitude,double alpha) {
	glEnable(GL_BLEND);
	GLfloat dif[] = { 0.2f, 0.2f, 0.9f, (float)alpha };
	GLfloat amb[] = { 0.2f, 0.2f, 0.2f, (float)alpha };
	GLfloat spe[] = { 1.0f, 1.0f, 1.0f, (float)alpha };
	GLfloat emi[] = { 0.0f, 0.0f, 0.0f, 0.0f };
	GLfloat shi = 50;
	glRotated(90.0, 0, 0, 1);
	glPushMatrix();
	{
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, dif);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spe);
		glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emi);
		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shi);
		glRotated(90.0, 1, 0, 0);
		glutSolidCylinder(arrowRadius, arrowHeight * amplitude, 20, 20);
	}
	glPopMatrix();

	glPushMatrix();
	{
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, dif);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spe);
		glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emi);
		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shi);
		glTranslated(0, -arrowHeight * amplitude, 0);
		glRotated(90.0, 1, 0, 0);
		glutSolidCone(coneRadius, coneHeight, 20, 20);
	}
	glPopMatrix();
	glDisable(GL_BLEND);
}

void drawBothArrow(bool isWP) {
	glPushMatrix();
	{
		glRotated(90.0, 0, 0, 1);
		if (!isWP) {
			GLfloat dif[] = { 0.8f, 0.2f, 0.2f, 1.0f };
			GLfloat amb[] = { 0.8f, 0.2f, 0.2f, 1.0f };
			GLfloat spe[] = { 1.0f, 1.0f, 1.0f, 1.0f };
			GLfloat emi[] = { 0.0f, 0.0f, 0.0f, 0.0f };
			GLfloat shi = 50;
			glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, dif);
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
			glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spe);
			glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emi);
			glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shi);
		} else {
			GLfloat dif[] = { 0.8f, 0.8f, 0.2f, 1.0f };
			GLfloat amb[] = { 0.8f, 0.8f, 0.2f, 1.0f };
			GLfloat spe[] = { 1.0f, 1.0f, 1.0f, 1.0f };
			GLfloat emi[] = { 0.0f, 0.0f, 0.0f, 0.0f };
			GLfloat shi = 50;
			glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, dif);
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
			glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spe);
			glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emi);
			glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shi);
		}


		double bothArrowSize = sheetSize / 4;
		double coneSize = bothArrowSize / 2.5;
		glTranslated(sheetSize*3 /4, bothArrowSize/2, 0);
		glPushMatrix();
		{
			glRotated(90.0, 1, 0, 0);
			glutSolidCylinder(sheetSize / 20, bothArrowSize, 20, 20);
		}
		glPopMatrix();
		glPushMatrix();
		{
			glTranslated(0, -bothArrowSize, 0);
			glRotated(90.0, 1, 0, 0);
			glutSolidCone(coneSize, coneSize, 20, 20);
		}
		glPopMatrix();
		glPushMatrix();
		{
			glRotated(-90.0, 1, 0, 0);
			glutSolidCone(coneSize, coneSize, 20, 20);
		}
		glPopMatrix();
	}
	glPopMatrix();
}

void drawSphere(double amplitude, bool isBright,double scale) {
	glPushMatrix();
	{
		glRotated(90.0, 0, 0, 1);
		if (isBright) {
			GLfloat dif[] = { 1.0f, 1.0f, 1.0f, 1.0f };
			GLfloat amb[] = { 0.2f, 0.2f, 0.2f, 1.0f };
			GLfloat spe[] = { 1.0f, 1.0f, 1.0f, 1.0f };
			GLfloat emi[] = { 1.0f, 1.0f, 1.0f, 1.0f };
			GLfloat shi = 50;
			glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, dif);
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
			glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spe);
			glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emi);
			glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shi);
		} else {
			GLfloat dif[] = { 0.2f, 0.9f, 0.2f, 1.0f };
			GLfloat amb[] = { 0.2f, 0.2f, 0.2f, 1.0f };
			GLfloat spe[] = { 1.0f, 1.0f, 1.0f, 1.0f };
			GLfloat emi[] = { 0.0f, 0.0f, 0.0f, 0.0f };
			GLfloat shi = 50;
			glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, dif);
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
			glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spe);
			glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emi);
			glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shi);
		}
		glTranslated(0, -arrowHeight * amplitude, 0);
		glutSolidSphere(sphereRadius*scale, 20, 20);
	}
	glPopMatrix();
}

void drawAlphaSphere(double amplitude, double alpha) {
	glPushMatrix();
	{
		glRotated(90.0, 0, 0, 1);
		GLfloat dif[] = { 0.2f, 0.2f, 0.9f, (float)alpha };
		GLfloat amb[] = { 0.2f, 0.2f, 0.2f, (float)alpha };
		GLfloat spe[] = { 1.0f, 1.0f, 1.0f, (float)alpha };
		GLfloat emi[] = { 0.0f, 0.0f, 0.0f, 0.0f };
		GLfloat shi = 50;
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, dif);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spe);
		glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emi);
		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shi);
		glTranslated(0, -arrowHeight * amplitude, 0);
		glutSolidSphere(sphereRadius, 20, 20);
	}
	glPopMatrix();
}

void drawCircle() {
	glPushMatrix();
	{
		GLfloat dif[] = { 0.8f, 0.8f, 0.8f, 1.0f };
		GLfloat amb[] = { 1.0f, 1.0f, 1.0f, 1.0f };
		GLfloat spe[] = { 0.0f, 0.0f, 0.0f, 1.0f };
		GLfloat emi[] = { 0.0f, 0.0f, 0.0f, 1.0f };
		GLfloat shi = 50;
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, dif);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spe);
		glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emi);
		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shi);
		glNormal3d(0, 0, 1);
		glBegin(GL_POLYGON);
		int slice = 20;
		for (int i = 0; i < slice; i++) {
			glVertex3d(sphereRadius*cos(2 * pi/slice*i),0 , sphereRadius*sin(2 * pi/slice*i));
		}
		glEnd();
	}
	glPopMatrix();
}
void drawAlphaCircle(double alpha) {
	glPushMatrix();
	{
		GLfloat dif[] = { 0.8f, 0.8f, 0.8f, (float)alpha };
		GLfloat amb[] = { 1.0f, 1.0f, 1.0f, (float)alpha };
		GLfloat spe[] = { 0.0f, 0.0f, 0.0f, (float)alpha };
		GLfloat emi[] = { 0.0f, 0.0f, 0.0f, 0.0f };
		GLfloat shi = 50;
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, dif);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spe);
		glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emi);
		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shi);
		glNormal3d(0, 0, 1);
		glBegin(GL_POLYGON);
		int slice = 20;
		for (int i = 0; i < slice; i++) {
			glVertex3d(sphereRadius*cos(2 * pi / slice*i), 0, sphereRadius*sin(2 * pi / slice*i));
		}
		glEnd();
	}
	glPopMatrix();
}


void drawSheet(double size) {
	double sized = size + framelength;
	double sizes = size - framelength;
	glPushMatrix();
	{
		glEnable(GL_BLEND);
		GLfloat dif[] = { 0.8f, 0.8f, 0.8f, 0.6f };
		GLfloat amb[] = { 0.2f, 0.2f, 0.2f, 0.6f };
		GLfloat spe[] = { 0.0f, 0.0f, 0.0f, 1.0f };
		GLfloat emi[] = { 0.0f, 0.0f, 0.0f, 1.0f };
		GLfloat shi = 50;
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, dif);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spe);
		glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emi);
		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shi);
		glBegin(GL_QUADS);
		glNormal3d(0, 0, -1);
		glVertex3d(sizes, sizes, -frameoffset);
		glVertex3d(sizes, -sizes, -frameoffset);
		glVertex3d(-sizes, -sizes, -frameoffset);
		glVertex3d(-sizes, sizes, -frameoffset);
		glNormal3d(0, 0, 1);
		glVertex3d(sizes, sizes, frameoffset);
		glVertex3d(sizes, -sizes, frameoffset);
		glVertex3d(-sizes, -sizes, frameoffset);
		glVertex3d(-sizes, sizes, frameoffset);
		glEnd();
		glDisable(GL_BLEND);
	}
	glPopMatrix();
	glPushMatrix();
	{
		glEnable(GL_BLEND);
		GLfloat dif[] = { 0.8f, 0.8f, 0.8f, 0.9f };
		GLfloat amb[] = { 0.8f, 0.8f, 0.8f, 0.9f };
		GLfloat spe[] = { 0.0f, 0.0f, 0.0f, 1.0f };
		GLfloat emi[] = { 0.0f, 0.0f, 0.0f, 1.0f };
		GLfloat shi = 50;
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, dif);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spe);
		glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emi);
		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shi);
		glBegin(GL_POLYGON);
		glNormal3d(0, 0, -1);
		glVertex3d(sized, sized, 0);
		glVertex3d(sized, -sized, 0);
		glVertex3d(-sized, -sized, 0);
		glVertex3d(-sized, sized, 0);
		glEnd();
		glDisable(GL_BLEND);
	}
	glPopMatrix();
}
void drawPlate(double size,double thick) {
	glPushMatrix();
	{
		GLfloat dif[] = { 0.4f, 0.4f, 0.4f, 0.3f };
		GLfloat amb[] = { 0.2f, 0.2f, 0.2f, 0.3f };
		GLfloat spe[] = { 0.0f, 0.0f, 0.0f, 0.0f };
		GLfloat emi[] = { 0.0f, 0.0f, 0.0f, 0.0f };
		GLfloat shi = 0;
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, dif);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spe);
		glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emi);
		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shi);
		// drawS
		glBegin(GL_QUADS);
		glNormal3d(0, 0, -1);
		glVertex3d(size, size, thick);
		glVertex3d(-size, size, thick);
		glVertex3d(-size, -size, thick);
		glVertex3d(size, -size, thick);
		glNormal3d(0, 0, 1);
		glVertex3d(size, size, 0);
		glVertex3d(-size, size, 0);
		glVertex3d(-size, -size, 0);
		glVertex3d(size, -size, 0);
		glNormal3d(0, -1, 0);
		glVertex3d(size, -size, thick);
		glVertex3d(-size, -size, thick);
		glVertex3d(-size, -size, 0);
		glVertex3d(size, -size, 0);
		glNormal3d(0, 1, 0);
		glVertex3d(size, size, thick);
		glVertex3d(-size, size, thick);
		glVertex3d(-size, size, 0);
		glVertex3d(size, size, 0);
		glNormal3d(-1, 0, 0);
		glVertex3d(-size, size, 0);
		glVertex3d(-size, size, thick);
		glVertex3d(-size, -size, thick);
		glVertex3d(-size, -size, 0);
		glNormal3d(1, 0, 0);
		glVertex3d(size, size, thick);
		glVertex3d(size, size, 0);
		glVertex3d(size, -size, 0);
		glVertex3d(size, -size, thick);
		glEnd();
	}
	glPopMatrix();
}

void drawMeshedSheet(double size) {
	glEnable(GL_BLEND);
	glPushMatrix();
	{
		GLfloat dif[] = { 0.1f, 0.1f, 0.1f, 0.9f };
		GLfloat amb[] = { 0.6f, 0.6f, 0.6f, 0.9f };
		GLfloat spe[] = { 0.5f, 0.5f, 0.5f, 0.9f };
		GLfloat emi[] = { 0.0f, 0.0f, 0.0f, 0.0f };
		GLfloat shi = 50;
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, dif);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spe);
		glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emi);
		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shi);

		double px, py, amplitude;
		setElectricField(arrowCount*arrowInterval, &px, &py, &amplitude);

		glTranslated(0, 0, (arrowCount) / 2 * arrowInterval);
		double os = size / sheetSlice;
		glBegin(GL_QUADS);
		for (int i = -sheetSlice; i < sheetSlice; i++) {
			for (int j = -sheetSlice; j < sheetSlice; j++) {

				double decayimg = 8e2 * amplitude;
				if (decayimg == 0.0) {
					dif[1] = 0.1f;
					amb[1] = 0.6f;
				} else {
					dif[1] = (float)max(0.1, min(1.0, exp(-os*os*(i*i + j*j) / pow(decayimg, 2))));
					amb[1] = (float)max(0.6, min(1.0, exp(-os*os*(i*i + j*j) / pow(decayimg, 2))));
				}
				glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, dif);
				glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);

				glNormal3d(0, 0, -1);
				glVertex3d(os*i, os*j, 0);
				glVertex3d(os*i, os*(j - 1), 0);
				glVertex3d(os*(i - 1), os*(j - 1), 0);
				glVertex3d(os*(i - 1), os*j, 0);
			}
		}
		glEnd();
	}
	glPopMatrix();
	glDisable(GL_BLEND);
}


void procCommand() {
	// input light type
	if (command == "pulse") {
		t0 = currentTime;
		ls = Pulse;
	}
	if (command == "cw") {
		t0 = currentTime;
		ls = CW;
	}
	if (command == "off") {
		t0 = currentTime;
		ls = Off;
	}

	// vector expression
	if (command == "arrow") {
		vs = Arrow;
	}
	if (command == "point") {
		vs = Point;
	}
	if (command == "novector") {
		vs = NoVec;
	}

	// front polarizer
	if (command == "front") {
		frontFlag = true;
	}
	if (command == "nofront") {
		frontFlag = false;
	}


	// animation progress
	if (command == "stop") {
		stop = true;
	}
	if (command == "start") {
		stop = false;
	}
	if (command == "fast") {
		animeSpeed = fastAnimeSpeed;
	}
	if (command == "normal") {
		animeSpeed = normalAnimeSpeed;
	}
	if (command == "slow") {
		animeSpeed = slowAnimeSpeed;
	}

	// input polarization
	if (command == "ph") {
		theta = 0;
		phi = 0;
		polarizerRotateFlag0 = false;
	}
	if (command == "pv") {
		theta = pi / 2;
		phi = 0;
		polarizerRotateFlag0 = false;
	}
	if (command == "pc") {
		theta = pi / 4;
		phi = pi / 2;
		polarizerRotateFlag0 = false;
	}
	if (command == "pcc") {
		theta = pi / 4;
		phi = -pi / 2;
		polarizerRotateFlag0 = false;
	}
	if (command == "pd") {
		theta = pi / 4;
		phi = 0;
		polarizerRotateFlag0 = false;
	}
	if (command == "pdd") {
		theta = -pi / 4;
		phi = 0;
		polarizerRotateFlag0 = false;
	}
	if (command == "protate") {
		polarizerRotateFlag0 = true;
	}
	if (command == "noprotate") {
		polarizerRotateFlag0 = false;
	}

	// polarizer1 angle
	if (command == "nop1") {
		polarizerFlag1 = false;
		polarizerRotateFlag1 = false;
	}
	if (command == "p1h") {
		polarizerTheta1 = 0;
		polarizerFlag1 = true;
		polarizerRotateFlag1 = false;
	}
	if (command == "p1v") {
		polarizerTheta1 = pi / 2;
		polarizerFlag1 = true;
		polarizerRotateFlag1 = false;
	}
	if (command == "p1d") {
		polarizerTheta1 = pi / 4;
		polarizerFlag1 = true;
		polarizerRotateFlag1 = false;
	}
	if (command == "p1dd") {
		polarizerTheta1 = -pi / 4;
		polarizerFlag1 = true;
		polarizerRotateFlag1 = false;
	}
	if (command == "p1rotate") {
		polarizerRotateFlag1 = true;
	}
	if (command == "nop1rotate") {
		polarizerRotateFlag1 = false;
	}

	// polarizer2 angle
	if (command == "nop2") {
		polarizerFlag2 = false;
		polarizerRotateFlag2 = false;
	}
	if (command == "p2h") {
		polarizerTheta2 = 0;
		polarizerFlag2 = true;
		wavePlateFlag = false;
		polarizerRotateFlag2 = false;
	}
	if (command == "p2v") {
		polarizerTheta2 = pi / 2;
		polarizerFlag2 = true;
		wavePlateFlag = false;
		polarizerRotateFlag2 = false;
	}
	if (command == "p2d") {
		polarizerTheta2 = pi / 4;
		polarizerFlag2 = true;
		wavePlateFlag = false;
		polarizerRotateFlag2 = false;
	}
	if (command == "p2dd") {
		polarizerTheta2 = -pi / 4;
		polarizerFlag2 = true;
		wavePlateFlag = false;
		polarizerRotateFlag2 = false;
	}
	if (command == "p2rotate") {
		polarizerRotateFlag2 = true;
	}
	if (command == "nop2rotate") {
		polarizerRotateFlag2 = false;
	}

	// waveplate angle
	if (command == "nowp") {
		wavePlateFlag = false;
		wavePlatePhase = 0;
	}
	if (command == "hwp") {
		wavePlateFlag = true;
		polarizerFlag2 = false;
		wavePlatePhase = pi;
	}
	if (command == "qwp") {
		wavePlateFlag = true;
		polarizerFlag2 = false;
		wavePlatePhase = pi / 2;
	}
	if (command == "wrotate") {
		wavePlateRotateFlag = true;
	}
	if (command == "nowrotate") {
		wavePlateRotateFlag = false;
	}
	if (command == "wh") {
		wavePlateTheta = 0;
		wavePlateFlag = true;
		wavePlateRotateFlag = false;
	}
	if (command == "wv") {
		wavePlateTheta = pi / 2;
		wavePlateFlag = true;
		wavePlateRotateFlag = false;
	}
	if (command == "wd") {
		wavePlateTheta = pi / 4;
		wavePlateFlag = true;
		wavePlateRotateFlag = false;
	}
	if (command == "wdd") {
		wavePlateTheta = -pi / 4;
		wavePlateFlag = true;
		wavePlateRotateFlag = false;
	}

	// compare vector without waveplate
	if (command == "ghost") {
		ghostArrowFlag = true;
	}
	if (command == "noghost") {
		ghostArrowFlag = false;
	}

	// draw projected electric field
	if (command == "projection") {
		projectionFlag = true;
	}
	if (command == "noprojection") {
		projectionFlag = false;
	}

	// draw current command
	if (command == "nostring") {
		stringFlag = false;
	}
	if (command == "string") {
		stringFlag = true;
	}

#ifdef USE_OPENCV

	// start recording video upto 120sec
	if (command == "record") {
		dataBuffer = (GLubyte*)malloc(width*height*cc);
		vw = cvCreateVideoWriter("cap.avi", -1, 30.0-0.03, cvSize(width, height));
		recordFlag = true;
		recordCount = 0;
		recordTime = 120.0;
	}

	// start recording 3sec video
	if (command == "record3") {
		dataBuffer = (GLubyte*)malloc(width*height*cc);
		vw = cvCreateVideoWriter("cap.avi", -1, videoFps, cvSize(width, height));
		recordFlag = true;
		recordCount = 0;
		recordTime = 3.0;
	}
	// stop recording video
	if (command == "stoprecord") {
		if (vw != NULL) {
			cvReleaseVideoWriter(&vw);
			vw = NULL;
			free(dataBuffer);
		}
		recordFlag = false;
	}
#endif //USE_OPENCV

	command = "";
}
