
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
const double pi = acos(0.0) * 2;

bool sw = false;

/* initialize */
bool camcont = false;
double currentTime = 0;
int lastMx = 0;
int lastMy = 0;
bool firstMouse = true;
string command = "";
bool arrowFlag = false;
bool stop = false;
bool pulseFlag = false;
bool offFlag = false;
LightState ls = CW;
double t0 = -10000;
double delta = 1e-10;
bool projectionFlag = false;
bool stringFlag = false;


#ifdef USE_OPENCV
bool recordFlag = false;
const unsigned int cc = 3;
void* dataBuffer = NULL;
IplImage *frame;
CvVideoWriter* vw;
double recordTime = 1000.0;
int recordCount = 0;
#endif //USE_OPENCV


// window
int width = 800;
int height = 600;
int fps = 60;
string windowTitle = "Polarization";

// camera position
double camtheta = -pi / 4; // rad
double camphi = pi * 7 / 16; // rad
double camdist = 10000; // (nm)
double camSpeed = 0.01; // (rad/pix)
double camscaling = 1.05;
double camFar = 30000; // (nm)
double camNear = 100; // (nm)
double camFov = 30.0; // (degree)
double camsmooththeta = camtheta;
double camsmoothphi = camphi;
double camsmoothdist = camdist;
double camMinSpeed = pi / 1e4;

// anime speed
double fastAnimeSpeed = 1e-4;
double normalAnimeSpeed = 3e-6;
double slowAnimeSpeed = 1e-6;
double animeSpeed = normalAnimeSpeed; // (ps per realSec) 1e-12
// double animeSpeed = 1; // (ps per realSec) 1e-12

// physical value
double lightSpeed = 3e5; //(m/ms) 1e-3
double waveLength = 780; // (nm) 1e-9
double lightFreq = lightSpeed / waveLength; // (THz) 1e12

// mesh ground
double meshInterval = 200; //(nm)
int meshCountZ = (int)(2 * waveLength / meshInterval); // num
int meshCountX = 10; // num
double meshZShift = -waveLength*1.5; // (nm)

// arrows
//double arrowInterval = waveLength / 18; //(nm)
double arrowInterval = waveLength / 10; //(nm)
int arrowCount = (int)(meshCountZ * 2 * meshInterval / arrowInterval); // num
double arrowHeight = waveLength / 3; // (nm)
double arrowRadius = arrowInterval / 10; // (nm)
double coneHeight = arrowRadius * 2; // (nm)
double coneRadius = arrowRadius * 2; // (nm)
double sphereRadius = coneRadius * 2; // (nm)
bool ghostArrowFlag = false;

// sheet
int sheetSlice = 100; // num
double sheetSize = arrowHeight*1.5; // (nm)
double spotSize = 6.0;

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
double polarizerRotateSpeed = 0.005; // (rad)
double polarizerPos1 = 4 * meshCountZ*meshInterval / 3; // (nm)
double polarizerPos2 = 2 * meshCountZ*meshInterval / 3; // (nm)

// waveplate
bool wavePlateFlag = false;
bool wavePlateRotateFlag = false;
double wavePlateTheta = 0;
double wavePlateLength = waveLength / 1; //(nm)
double wavePlatePhase = pi * 2; //(rad)

// screen object
unsigned int historyCount = 30;
vector<pair<double, double > > history;

// decay speed
double pulseWidth = 4 * waveLength / lightSpeed; // (ps) gaussian sigma
double pulsePower = 1e0; // (nm) gaussian amplitude
double dipoleDisp = arrowHeight; // (nm) dipole amplitude
double decaySpeed = 1e-3; // (ps) speed of switch
double polarizerDecayLength = meshInterval / 2; // (nm)


double gaussian(double t) {
	return pulsePower * exp(-pow(t - (t0 + pulseWidth * 3), 2) / 2 / pow(pulseWidth, 2));
}
double expdecay(double t) {
	return exp(-max(0.0, (t - t0)) / decaySpeed);
}
double expup(double t) {
	return 1 - exp(-max(0.0, (t - t0)) / decaySpeed);
}

void setElectricField(double pos, double* vx, double* vy, double* amplitude) {

	// first value
	double lightPhase = (pos / waveLength - currentTime*lightFreq) * 2 * pi;
	complex<double> i = complex<double>(0, 1);
	complex<double> px = cos(theta)*exp(i*lightPhase);
	complex<double> py = sin(theta)*exp(i*(lightPhase + phi));
	complex<double> ppx, ppy;

	// process polarizer 2
	if (polarizerFlag2) {
		ppx = px*cos(polarizerTheta2) + py*sin(polarizerTheta2);
		ppy = -px*sin(polarizerTheta2) + py*cos(polarizerTheta2);
		ppy *= 2.0 / (exp(max(0.0, pos - polarizerPos2) / polarizerDecayLength) + 1);
		px = ppx*cos(polarizerTheta2) - ppy * sin(polarizerTheta2);
		py = ppx*sin(polarizerTheta2) + ppy * cos(polarizerTheta2);
	}

	// process waveplate
	if (wavePlateFlag) {
		ppx = px*cos(wavePlateTheta) + py*sin(wavePlateTheta);
		ppy = -px*sin(wavePlateTheta) + py*cos(wavePlateTheta);
		ppy *= exp(i*min(max(0.0, pos - polarizerPos2), wavePlateLength) / wavePlateLength*wavePlatePhase);
		px = ppx*cos(wavePlateTheta) - ppy * sin(wavePlateTheta);
		py = ppx*sin(wavePlateTheta) + ppy * cos(wavePlateTheta);
	}

	// process polarizer 1
	if (polarizerFlag1) {
		ppx = px*cos(polarizerTheta1) + py*sin(polarizerTheta1);
		ppy = -px*sin(polarizerTheta1) + py*cos(polarizerTheta1);
		ppy *= 2.0 / (exp(max(0.0, pos - polarizerPos1) / polarizerDecayLength) + 1);
		px = ppx*cos(polarizerTheta1) - ppy * sin(polarizerTheta1);
		py = ppx*sin(polarizerTheta1) + ppy * cos(polarizerTheta1);
	}


	// calc beam shape
	double decay = 0;
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

	// convert to real value
	(*vx) = real(px);
	(*vy) = real(py);
	(*amplitude) = sqrt(norm(px) + norm(py));
	return;
}

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

void display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadIdentity();

	// locate camera
	double cphi = min(max(camsmoothphi, delta), pi - delta);
	double ctheta = camsmooththeta;
	double eyex = camsmoothdist*cos(ctheta)*sin(cphi);
	double eyez = camsmoothdist*sin(ctheta)*sin(cphi);
	double eyey = camsmoothdist*cos(cphi);
	gluLookAt(eyex, eyey, eyez, 0, 0, 0, 0, 1, 0);

	double ratio = 0.2;
	if (fabs(camsmooththeta - camtheta) < camMinSpeed) {
		camsmooththeta = camtheta;
	} else {
		camsmooththeta = ratio*camtheta + (1 - ratio)*camsmooththeta;
	}
	if (fabs(camsmoothphi - camphi) < camMinSpeed) {
		camsmoothphi = camphi;
	} else {
		camsmoothphi = ratio*camphi + (1 - ratio)*camsmoothphi;
	}
	if (fabs(camsmoothdist - camdist) < camMinSpeed) {
		camsmoothdist = camdist;
	} else {
		camsmoothdist = ratio*camdist + (1 - ratio)*camsmoothdist;
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

	int warray = 20;
	if (sw) warray = 1;
	// draw electric field
	for (int j = 0; j < warray; j++) {
		for (int i = 0; i <= arrowCount; i++) {
			glPushMatrix();
			{
				setElectricField(i*arrowInterval, &px, &py, &amplitude);
				double length = hypot(px, py)/2;
				double rot = atan2(py, px);
				glTranslated((j-warray/2)*2*arrowInterval, 0, (i - arrowCount / 2)*arrowInterval);
				if(sw) glRotated(rot / 2 / pi * 360, 0, 1, 0);
				else glRotated(rot / 2 / pi * 360, 0, 0, 1);
				if (arrowFlag) {
					drawArrow(length);
				} else {
					drawSphere(length, (i == 0 || i == arrowCount), 1.0);
				}
				if (i == arrowCount) {
					history.push_back(make_pair(px, py));
					if (history.size()>historyCount) {
						history.erase(history.begin());
					}
				}
			}
			glPopMatrix();
		}
	}


	glFlush();
	glutSwapBuffers();

#ifdef USE_OPENCV
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
		recordCount++;

		if (1.0*recordCount / fps > recordTime) {
			cvReleaseVideoWriter(&vw);
			vw = NULL;
			free(dataBuffer);
			recordFlag = false;
		}
	}
#endif //USE_OPENCV
}

void timer(int value) {
	glutTimerFunc(1000 / fps, timer, 0);
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
	if ('a' <= key && key <= 'z' || key == '=' || '0' <= key && key <= '9') {
		command += key;
	}
	if (key == VK_SPACE) {
		glutFullScreenToggle();
	}
	if (key == VK_BACK) {
		if (command.length() > 0) {
			command = command.substr(0, command.length() - 1);
		}
	}
	if (key == 'W') {
		camphi -= pi / 32;
		if (camphi < 0)camphi = 0;
	}
	if (key == 'S') {
		camphi += pi / 32;
		if (camphi > pi)camphi = pi;
	}
	if (key == 'D') {
		camtheta += pi / 16;
	}
	if (key == 'A') {
		camtheta -= pi / 16;
	}
	if (key == 'F') {
		camdist *= 1.05;
	}
	if (key == 'R') {
		camdist /= 1.05;
	}
	if (key == 'I') {
		camtheta = 0;
		camphi = 0;
	}
	if (key == 'K') {
		camtheta = 0;
		camphi = pi / 2;
	}
	if (key == 'J') {
		camtheta = pi / 2;
		camphi = pi / 2;
	}
	if (key == 'L') {
		camtheta = -pi / 2;
		camphi = pi / 2;
	}
	if (key == 'U') {
		camtheta = pi * 3 / 16;
		camphi = pi * 6 / 16;
	}
	if (key == 'O') {
		camtheta = -pi * 3 / 16;
		camphi = pi * 6 / 16;
	}

	if (key == VK_RETURN) {
		procCommand();
		command = "";
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
void mouseWheel(int wheelNumber, int direction, int x, int y) {
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
		} else if (state == GLUT_DOWN) {
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


int main(int argc, char** argv) {
	glutInit(&argc, argv);;
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(width, height);
	glutCreateWindow(windowTitle.c_str());

	// callback
	glutDisplayFunc(display);
	glutTimerFunc(1000 / fps, timer, 0);
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
		GLfloat lightpos[] = { 0.0f, 1000.0f, 0.0f, 0.0f };
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
		for (int p = -meshCountX + 1; p < meshCountX; p++) {
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
		for (int p = -meshCountZ + 1; p < meshCountZ; p++) {
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
			for (int p = -meshCountX + 1; p < meshCountX; p++) {
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
			for (int p = -meshCountZ + 1; p < meshCountZ; p++) {
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
		GLfloat dif[] = { 0.2f, 0.2f, 0.9f, 1.0f };
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

void drawAlphaArrow(double amplitude, double alpha) {
	glEnable(GL_BLEND);
	GLfloat dif[] = { 0.2f, 0.9f, 0.2f, (float)alpha };
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
		glTranslated(sheetSize * 3 / 4, bothArrowSize / 2, 0);
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

void drawSphere(double amplitude, bool isBright, double scale) {
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
			GLfloat dif[] = { 0.1f, 0.1f, 0.8f, 1.0f };
			GLfloat amb[] = { 0.2f, 0.2f, 0.8f, 1.0f };
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
		glScaled(3.0, 0.2, 3.0);
		glutSolidSphere(sphereRadius*scale, 20, 20);
	}
	glPopMatrix();
}

void drawAlphaSphere(double amplitude, double alpha) {
	glPushMatrix();
	{
		glRotated(90.0, 0, 0, 1);
		GLfloat dif[] = { 0.2f, 0.9f, 0.2f, (float)alpha };
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
			glVertex3d(sphereRadius*cos(2 * pi / slice*i), 0, sphereRadius*sin(2 * pi / slice*i));
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
		glNormal3d(0, 0, -1);
		glBegin(GL_POLYGON);
		glVertex3d(size, size, 0);
		glVertex3d(size, -size, 0);
		glVertex3d(-size, -size, 0);
		glVertex3d(-size, size, 0);
		glEnd();
		glDisable(GL_BLEND);
	}
	glPopMatrix();
}
void drawPlate(double size, double thick) {
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
	// input figure
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

	// arrow picture
	if (command == "arrow") {
		arrowFlag = true;
	}
	if (command == "noarrow") {
		arrowFlag = false;
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
	if (command == "pv") {
		theta = 0;
		phi = 0;
		polarizerRotateFlag0 = false;
	}
	if (command == "ph") {
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
	if (command == "p1v") {
		polarizerTheta1 = 0;
		polarizerFlag1 = true;
		polarizerRotateFlag1 = false;
	}
	if (command == "p1h") {
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
	if (command == "p2v") {
		polarizerTheta2 = 0;
		polarizerFlag2 = true;
		wavePlateFlag = false;
		polarizerRotateFlag2 = false;
	}
	if (command == "p2h") {
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

	// waveplate 
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
	if (command == "wv") {
		wavePlateTheta = 0;
		wavePlateFlag = true;
		wavePlateRotateFlag = false;
	}
	if (command == "wh") {
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

	if (command == "ghost") {
		ghostArrowFlag = true;
	}
	if (command == "noghost") {
		ghostArrowFlag = false;
	}
	if (command == "projection") {
		projectionFlag = true;
	}
	if (command == "noprojection") {
		projectionFlag = false;
	}

	if (command == "nostring") {
		stringFlag = false;
	}
	if (command == "string") {
		stringFlag = true;
	}

#ifdef USE_OPENCV
	if (command == "record") {

		dataBuffer = (GLubyte*)malloc(width*height*cc);
		//		cvCreateVideoWriter("cap.avi", CV_FOURCC('P', 'I', 'M', '1'), 15, cvSize(width, height));
		vw = cvCreateVideoWriter("cap.avi", -1, 30.0 - 0.03, cvSize(width, height));
		recordFlag = true;
		recordCount = 0;
	}
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
