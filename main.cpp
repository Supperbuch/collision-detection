#include <iostream>
#include "lineSegmentsIntersection.h"
#include "OBB 1.0.h"

using namespace std;


bool mouseDown = false;

float xrot = 0.0f;
float yrot = 0.0f;

float xdiff = 0.0f;
float ydiff = 0.0f;

void resize(int w, int h) {
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	glViewport(0, 0, w, h);

	gluPerspective(45.0f, 1.0f * w / h, 1.0f, 100.0f);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void mouse(int button, int state, int x, int y) {
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
		mouseDown = true;
		xdiff = x - yrot;
		ydiff = -y + xrot;
		//std::cout << "xdiff:" << xdiff << "\tydiff" << ydiff << std::endl;
	}
	else
		mouseDown = false;
}

void mouseMotion(int x, int y) {
	if (mouseDown) {
		yrot = x - xdiff;
		xrot = y + ydiff;
		//std::cout << "yrot:" << yrot << "\txrot" << xrot << std::endl;

		glutPostRedisplay();
	}
}

void display() {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	gluLookAt(
		0.0f, 0.0f, 3.0f,
		0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f);

	// Realize the  mouse rotation
	glRotatef(xrot, 1.0f, 0.0f, 0.0f);
	glRotatef(yrot, 0.0f, 1.0f, 0.0f);


	//world coordinates
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glLineWidth(1.0f);

	glColor3f(1, 0, 0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0.8, 0, 0);

	glColor3f(0, 1, 0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0.8, 0);

	glColor3f(0, 0, 1);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, 0.8);

	glEnd();

	/************************************************************************************/
	float length = 20;
	float width = 10;
	float height = 5;


	//OBB boundary cases
	/*float length = 20;
	float width = 8;
	float height = 4;*/

	//face face
	//Quaternionf quaternionPre1(1, 0, 0, 0);
	//Vector3f centerP1(0, 0, 2);
	//Quaternionf quaternion1(1, 0, 0, 0);
	//Vector3f vel_1(0, 0, 3);

	//Quaternionf quaternionPre2(sqrt(3) / 2, 0, 0, 0.5);
	//Vector3f centerP2(0, 0, 6);
	//Quaternionf quaternion2(sqrt(2) / 2, 0, 0, sqrt(2) / 2);
	////Quaternionf quaternion2(1, 0, 0, 0);
	//Vector3f vel_2(0, 0, -5);

	/************************************************************************************/
	//draw the move direction of center point
	/*glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glColor3f(1, 0, 0);
	glLineWidth(2.0);
	glBegin(GL_LINES);
	glVertex3f(centerP1(0)/20, centerP1(1)/20, centerP1(2)/20);
	glVertex3f((centerP1(0)/20 + vel_1(0)/5), (centerP1(1) / 20 + vel_1(1)/5), (centerP1(2) / 20 + vel_1(2)/5));
	glEnd();
	
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glColor3f(1, 0, 0);
	glPointSize(5.0f);
	glBegin(GL_POINTS);
	glVertex3f((centerP1(0) / 20 + vel_1(0) / 5), (centerP1(1) / 20 + vel_1(1) / 5), (centerP1(2) / 20 + vel_1(2) / 5));
	glEnd();*/
	/***********************************************************************************/

	//point point
	/*Quaternionf quaternionPre1(1, 0, 0, 0);
	Vector3f centerP1(0, 0, 2);
	Quaternionf quaternion1(1, 0, 0, 0);
	Vector3f vel_1(1, 1, 1);

	Quaternionf quaternionPre2(1, 0, 0, 0);
	Vector3f centerP2(20, 8, 6);
	Quaternionf quaternion2(1, 0, 0, 0);
	Vector3f vel_2(0.5, 0.5, 0.5);*/


	//edge edge
	/*Quaternionf quaternionPre1(1, 0, 0, 0);
	Vector3f centerP1(0, 0, 2);
	Quaternionf quaternion1(1, 0, 0, 0);
	Vector3f vel_1(1, 1, 1);

	Quaternionf quaternionPre2(sqrt(3) / 2, 0, 0.5, 0);
	Vector3f centerP2(6.732, 4, 13.66025);
	Quaternionf quaternion2(1, 0, 0, 0);
	Vector3f vel_2(0.5, 0.5, 0.5);*/


	//edge face
	/*Quaternionf quaternionPre1(1, 0, 0, 0);
	Vector3f centerP1(0, 0, 2);
	Quaternionf quaternion1(1, 0, 0, 0);
	Vector3f vel_1(0, 0, 1);

	Quaternionf quaternionPre2(sqrt(3) / 2, 0, 0.5, 0);
	Vector3f centerP2(0, 0, 5+5*sqrt(3));
	Quaternionf quaternion2(1, 0, 0, 0);
	Vector3f vel_2(0, 0, -5.5);*/


	//point face
	/*Vector3f eulerAngle(0.5, 2, -0.5);

	Eigen::AngleAxisf rollAngle(AngleAxisf(eulerAngle(2), Vector3f::UnitX()));
	Eigen::AngleAxisf pitchAngle(AngleAxisf(eulerAngle(1), Vector3f::UnitY()));
	Eigen::AngleAxisf yawAngle(AngleAxisf(eulerAngle(0), Vector3f::UnitZ()));

	Quaternionf quaternionPre1(1, 0, 0, 0);
	Vector3f centerP1(0, 0, 2);
	Quaternionf quaternion1(1, 0, 0, 0);
	Vector3f vel_1(0, 0, 1);

	Quaternionf quaternionPre2;
	quaternionPre2 = yawAngle * pitchAngle*rollAngle;
	Vector3f centerP2(0, 0, 14.621426);
	Quaternionf quaternion2(sqrt(3) / 2, 0, 0.5, 0);
	Vector3f vel_2(0, 0, -20);*/

	//point edge
	/*Vector3f eulerAngle(0.5, 2, -0.5);

	Eigen::AngleAxisf rollAngle(AngleAxisf(eulerAngle(2), Vector3f::UnitX()));
	Eigen::AngleAxisf pitchAngle(AngleAxisf(eulerAngle(1), Vector3f::UnitY()));
	Eigen::AngleAxisf yawAngle(AngleAxisf(eulerAngle(0), Vector3f::UnitZ()));

	Quaternionf quaternionPre1(1, 0, 0, 0);
	Vector3f centerP1(0, 0, 2);
	Quaternionf quaternion1(1, 0, 0, 0);
	Vector3f vel_1(0, 0, 1);

	Quaternionf quaternionPre2;
	quaternionPre2 = yawAngle * pitchAngle*rollAngle;
	Vector3f centerP2(0, -1.3669, 14.621426);
	Quaternionf quaternion2(sqrt(3) / 2, 0, 0.5, 0);
	Vector3f vel_2(0, 0, -20);*/


	//general case
	//Vector3f eulerAngle(0.5, 0.5, -0.5); //[-2¦Ð£¬ 2¦Ð]

	//Eigen::AngleAxisf rollAngle(AngleAxisf(eulerAngle(2), Vector3f::UnitX()));
	//Eigen::AngleAxisf pitchAngle(AngleAxisf(eulerAngle(1), Vector3f::UnitY()));
	//Eigen::AngleAxisf yawAngle(AngleAxisf(eulerAngle(0), Vector3f::UnitZ()));

	//Quaternionf quaternionPre1(1, 0, 0, 0);
	//Vector3f centerP1(0, 0, 50);
	//Quaternionf quaternion1(sqrt(3) / 2, 0, 0, 0.5);
	//Vector3f vel_1(8.5, 1, 5);

	//Quaternionf quaternionPre2;
	//quaternionPre2 = yawAngle * pitchAngle*rollAngle;
	//Vector3f centerP2(0, -1.3669, 14.621426);
	//Quaternionf quaternion2(sqrt(3) / 2, 0, 0.5, 0);
	//Vector3f vel_2(0, 1, -15);

	//general case( debug, interval of A contain B, shared plane should be modified, have done
	/*Quaternionf quaternionPre1(-0.5029, 0.0481, 0.7072, 0.4946);
	Vector3f centerP1(20, 17, 3);
	Quaternionf quaternion1(-0.1408, -0.2478, 0.2848, -0.9152);
	Vector3f vel_1(10, 9, -17);

	Quaternionf quaternionPre2(0.5049, -0.2959, 0.7886, 0.1886);
	Vector3f centerP2(15, 13, 6);
	Quaternionf quaternion2(0.7013, -0.3802, -0.2415, -0.5525);
	Vector3f vel_2(5, 25, 24);*/

	
	
	//test  edge-edge after separation
	Quaternionf quaternionPre1(-0.681018, 0.193147, 0.595749, 0.379463);
	Vector3f centerP1(15, 14, 14);
	Quaternionf quaternion1(-0.138056, -0.786585, 0.15255, -0.582197);
	Vector3f vel_1(19, 8, -10);

	Quaternionf quaternionPre2(0.850016, -0.245109, 0.28046, 0.372473);
	Vector3f centerP2(15, 6, 17);
	Quaternionf quaternion2(-0.6978, -0.210287, -0.622866, 0.284416);
	Vector3f vel_2(-4, 18, 20);


	

	
	/***************************************************************************************/

	//start pose
	/*OBB OBBParamter1(centerP1, centerP2, quaternionPre1, quaternionPre2, length, width, height);
	OBBParamter1.FindCollisionFeatures();
	cout << "---------------------------" << endl;*/



	Vector3f color1(0, 0, 1);
	Vector3f color2(0, 1, 0);
	float lineWidth1 = 2;

	//Validation, move object2 along normal, until they separate
	VectorXf A;
	OBB Validation1(centerP1, centerP2, quaternionPre1, quaternionPre2, length, width, height);
	A = Validation1.result();
	if (A(0) != 0)
	{
		Vector3f V1(A(0)*A(1), A(0)*A(2), A(0)*A(3));
		Vector3f normal(A(1), A(2), A(3));
		OBB NormalDirection(centerP1, centerP2 + V1, quaternionPre1, quaternionPre2, length, width, height);
		if (NormalDirection.CorrectNormal(normal))
		{
			//draw the separated object
			OBB VisualValidation1(centerP1, centerP2 + V1, quaternionPre1, quaternionPre2, length, width, height);
			VisualValidation1.drawBox(color1, color2, lineWidth1);

			//draw real distance
			OBB Validationresult1(centerP1, centerP2 + V1, quaternionPre1, quaternionPre2, length, width, height);
			Validationresult1.DistanceOfBoxesVerification();
		}
		else
		{
			//draw the separated object
			OBB VisualValidation1(centerP1, centerP2 - V1, quaternionPre1, quaternionPre2, length, width, height);
			VisualValidation1.drawBox(color1, color2, lineWidth1);

			//draw real distance
			OBB Validationresult1(centerP1, centerP2 - V1, quaternionPre1, quaternionPre2, length, width, height);
			Validationresult1.DistanceOfBoxesVerification();
		}
		
	}


	//draw the start pose of objects
	/*OBB Visual1(centerP1, centerP2, quaternionPre1, quaternionPre2, length, width, height);
	Visual1.drawBox(color1, color2, lineWidth1);*/

	//project the interval of vertices to normal 
	OBB Point1(centerP1, centerP2, quaternionPre1, quaternionPre2, length, width, height);
	Point1.drawProjectPoint();

	/***************************************************************************************/

	/*LineSegmentsIntersection3D LineParameter(centerP1, vel_1, quaternionPre1, quaternion1, centerP2, vel_2, quaternionPre2, quaternion2, length, width, height);
	LineParameter.HandleGeneralCase();
	cout << "---------------------------" << endl;*/

	/***************************************************************************************/


	//end pose
	//Vector3f CP1_end;
	//CP1_end = centerP1 + vel_1;
	//Vector3f CP2_end;
	//CP2_end = centerP2 + vel_2;

	//Quaternionf quaternion1_end;
	//quaternion1_end = quaternionPre1 * quaternion1;
	//Quaternionf quaternion2_end;
	//quaternion2_end = quaternionPre2 * quaternion2;


	///*OBB OBBParamter2(CP1_end, CP2_end, quaternion1_end, quaternion2_end, length, width, height);
	//OBBParamter2.FindCollisionFeatures();
	//cout << "---------------------------" << endl;*/

	//Vector3f color3(0, 0, 1);
	//Vector3f color4(0, 1, 0);
	//float lineWidth2 = 2;

	//VectorXf B;
	//OBB Validation2(CP1_end, CP2_end, quaternion1_end, quaternion2_end, length, width, height);
	//B = Validation2.result();
	//if (B(0) != 0)
	//{
	//	Vector3f V2(B(0)*B(1), B(0)*B(2), B(0)*B(3));
	//	OBB VisualValidation2(CP1_end, CP2_end + V2, quaternion1_end, quaternion2_end, length, width, height);
	//	VisualValidation2.drawBox(color3, color4, lineWidth2);
	//}
	////draw the end pose of objects
	//OBB Visual2(CP1_end, CP2_end, quaternion1_end, quaternion2_end, length, width, height);
	//Visual2.drawBox(color3, color4, lineWidth2);

	//project the interval of vertices to normal 
	/*OBB Point2(CP1_end, CP2_end, quaternion1_end, quaternion2_end, length, width, height);
	Point2.drawProjectPoint();*/


	/***************************************************************************************/




	glFlush();
	glutSwapBuffers();
}


int main(int argc, char* argv[])
{
	
	glutInit(&argc, argv);

	glutInitWindowPosition(50, 50);
	glutInitWindowSize(800, 600);

	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);

	glutCreateWindow("cuboids in space");

	
	glutDisplayFunc(display);
	glutMouseFunc(mouse);
	glutMotionFunc(mouseMotion);
	glutReshapeFunc(resize);
		
	glClearColor(0.93f, 0.93f, 0.93f, 0.0f);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glClearDepth(1.0f);

	glutMainLoop();


	return (0);
}