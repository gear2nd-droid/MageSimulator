#include "SimpleGL.cuh"


// opengl
CameraParam cameraParam;
const float maxAngleX = M_PI - 0.01f;
const float minAngleX = -M_PI - 0.01f;
const float maxAngleZ = M_PI - 0.01f;
const float minAngleZ = -M_PI + 0.01f;
int lastMouseX;
int lastMouseY;
bool isLeftDragging = false;
bool isRightDragging = false;
int windowWidth = 640;
int windowHeight = 480;
int fpsCollision = 100;
int fpsDisplay = 10;

void init(int argc, char** argv)
{
	//extern int windowWidth;
	//extern int windowHeight;
	//extern int fps;
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(windowWidth, windowHeight);
	glutCreateWindow("OpenGL Mouse Control");
	glutDisplayFunc(sampling);
	glutTimerFunc(1000 / fpsCollision, timer, 0);
	glutReshapeFunc(reshape);
	glutMotionFunc(motion);
	glutMouseFunc(mouse);
	glutKeyboardFunc(keyboard);
	glutSpecialFunc(specialKeyboard);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_DEPTH_TEST);
	glutMainLoop();
}

void reshape(int w, int h) 
{
	windowWidth = w;
	windowHeight = h;
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0f, (float)w / (float)h, 0.1f, 1000.0f);
}

void motion(int x, int y) 
{
	int modifiers = glutGetModifiers();
	int deltaX = x - lastMouseX;
	int deltaY = y - lastMouseY;
	if (isLeftDragging) {
		if (modifiers & GLUT_ACTIVE_SHIFT) {
			const float sensitivity = 0.5f;
			cameraParam.cameraDistance += deltaY * sensitivity;
			if (cameraParam.cameraDistance < 1.0f) cameraParam.cameraDistance = 1.0f;
			if (cameraParam.cameraDistance > 500.0f) cameraParam.cameraDistance = 500.0f;
		}
		else {
			const float sensitivity = 0.005f;
			cameraParam.cameraAngleZ += deltaX * sensitivity;
			cameraParam.cameraAngleX += deltaY * sensitivity;
			if (cameraParam.cameraAngleX > maxAngleX) cameraParam.cameraAngleX = maxAngleX;
			if (cameraParam.cameraAngleX < minAngleX) cameraParam.cameraAngleX = minAngleX;
			if (cameraParam.cameraAngleZ > maxAngleZ) cameraParam.cameraAngleZ = maxAngleZ;
			if (cameraParam.cameraAngleZ < minAngleZ) cameraParam.cameraAngleZ = minAngleZ;
		}
		lastMouseX = x;
		lastMouseY = y;

		glutPostRedisplay();
	}

	if (isRightDragging) {
		if (modifiers & GLUT_ACTIVE_SHIFT) {
			const float sensitivity = 0.5f;
			cameraParam.targetPosZ += deltaY * sensitivity;
			if (cameraParam.targetPosZ < -500.0f) cameraParam.targetPosZ = -500.0f;
			if (cameraParam.targetPosZ > 500.0f) cameraParam.targetPosZ = 500.0f;
		}
		else {
			const float sensitivity = 0.5f;
			cameraParam.targetPosX += deltaX * sensitivity;
			cameraParam.targetPosY += deltaY * sensitivity;
			if (cameraParam.targetPosX > 500.0f) cameraParam.targetPosX = 500.0f;
			if (cameraParam.targetPosX < -500.0f) cameraParam.targetPosX = -500.0f;
			if (cameraParam.targetPosY > 500.0f) cameraParam.targetPosY = 500.0f;
			if (cameraParam.targetPosY < -500.0f) cameraParam.targetPosY = -500.0f;
		}

		lastMouseX = x;
		lastMouseY = y;

		glutPostRedisplay();
	}
}

void mouse(int button, int state, int x, int y) 
{
	extern bool isloop;
	if (button == GLUT_LEFT_BUTTON) {
		if (state == GLUT_DOWN) {
			isLeftDragging = true;
			isloop = false;
			lastMouseX = x;
			lastMouseY = y;
		}
		else {
			isLeftDragging = false;
			isloop = true;
		}
	}

	if (button == GLUT_RIGHT_BUTTON) {
		if (state == GLUT_DOWN) {
			isRightDragging = true;
			isloop = false;
			lastMouseX = x;
			lastMouseY = y;
		}
		else {
			isRightDragging = false;
			isloop = true;
		}
	}
}

void keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case '\033': // ESC
		extern int timeCnt;
		extern int printFullCnt;
		extern bool isloop;
		if (isloop)
		{
			isloop = false;
			char inputline[10];
			if (fgets(inputline, sizeof inputline, stdin) != NULL)
			{
				int val = atoi(inputline);
				if (val >= printFullCnt)
				{
					val = printFullCnt - 1;
				}
				else if (val < 0)
				{
					val = 0;
				}
				timeCnt = val;
				isloop = true;
			}
		}
		break;
	default:
		break;
	}
}

void specialKeyboard(int key, int x, int y) 
{
	extern std::vector<int> printHeader;
	extern PrintItem* print;
	extern int timeCnt;
	extern int printFullCnt;
	int preBlock, preLayer;
	switch (key) {
	case GLUT_KEY_UP:
	{
		for (int i = 0; i < printHeader.size(); i++)
		{
			if (printHeader[i] > timeCnt)
			{
				timeCnt = printHeader[i];
				break;
			}
			else if (i == printHeader.size() - 1)
			{
				timeCnt = printFullCnt - 1;
				break;
			}
		}
	}
		break;
	case GLUT_KEY_DOWN:
	{
		int prepre = 0;
		int pre = 0;
		for (int i = 0; i < printHeader.size(); i++)
		{
			if (pre < timeCnt && timeCnt <= printHeader[i])
			{
				timeCnt = prepre;
				break;
			}
			else if (i == printHeader.size() - 1)
			{
				timeCnt = prepre;
				break;
			}
			prepre = pre;
			pre = printHeader[i];
		}
	}
		break;
	case GLUT_KEY_LEFT:
	{
		int val = timeCnt - TIME_STEP_LEFT_RIGHT;
		if (val < 0)
		{
			val = 1;
		}
		timeCnt = val;
	}
		break;
	case GLUT_KEY_RIGHT:
	{
		int val = timeCnt + TIME_STEP_LEFT_RIGHT;
		if (val >= printFullCnt)
		{
			val = printFullCnt - 1;
		}
		timeCnt = val;
	}
		break;
	default:
		break;
	}
}

void timer(int value)
{
	glutPostRedisplay();
	glutTimerFunc(1000 / fpsCollision, timer, 0);
}

void drawCoordinate(float length, PrintItem item, Kinematics* kin)
{
	float mat[3 * 3];
	kin->GetRotationMatrix(item.cmdVal[3], item.cmdVal[4], item.cmdVal[5], mat);
	float bufVec[3];
	bufVec[0] = 1.0; bufVec[1] = 0.0; bufVec[2] = 0.0;
	float bufVecX[3];
	calcMatVec(mat, bufVec, bufVecX);
	bufVec[0] = 0.0; bufVec[1] = 1.0; bufVec[2] = 0.0;
	float bufVecY[3];
	calcMatVec(mat, bufVec, bufVecY);
	bufVec[0] = 0.0; bufVec[1] = 0.0; bufVec[2] = 1.0;
	float bufVecZ[3];
	calcMatVec(mat, bufVec, bufVecZ);

	glPushMatrix();
	glColor3d(1.0, 0.0, 0.0);
	glBegin(GL_LINES);
	glVertex3f(0.0, 0.0, 0.0);
	glVertex3f(bufVecX[0] * length, bufVecX[1] * length, bufVecX[2] * length);
	glEnd();
	glColor3d(0.0, 1.0, 0.0);
	glBegin(GL_LINES);
	glVertex3f(0.0, 0.0, 0.0);
	glVertex3f(bufVecY[0] * length, bufVecY[1] * length, bufVecY[2] * length);
	glEnd();
	glColor3d(0.0, 0.0, 1.0);
	glBegin(GL_LINES);
	glVertex3f(0.0, 0.0, 0.0);
	glVertex3f(bufVecZ[0] * length, bufVecZ[1] * length, bufVecZ[2] * length);
	glEnd();
	glPopMatrix();
}