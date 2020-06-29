/*******************************************************************************
	

		�����p���[�A�V�X�g���{�b�g��`�悷��v���O����
		
		����ҁ@hora
	�@�@������@2016/11/25
		�X�V���@2016/11/25

		�����\�z�@AntTweakBar��dll,lib,h�t�@�C��������̃t�H���_�ɓ����K�v����
					openGL�̊g���@�\�Ƃ���glut������K�v����
********************************************************************************/

#include "mainDraw.h"




/************************************************************
					�O���[�o���ϐ�
************************************************************/
double DSP_WIDTH  = 1200;	//�E�B���h�E�̉���
double DSP_HEIGHT = 1920;   //�E�B���h�E�̏c��

int time_count = 0; //���Ԃ��}�b�N�X�l�ɂȂ�����
double max = 0.0;
int max_num = 0;
int drawMode=0;
LARGE_INTEGER freq,start,end;

/************************************************************
					�c�[���o�[�p�̕ϐ�
************************************************************/
tweakHandle hBar;		//�c�[���o�[�̃n���h�����i�[���邽�߂̍\����
tweakParam  barParam;	//�c�[���o�[�̃p�����[�^���i�[���邽�߂̍\����	(main.cpp��main()����extern�ϐ��Ƃ��Ďg�p)

/************************************************************
					�f�[�^�i�[�p�̕ϐ�
************************************************************/
data_struct dataStr;	//dat�t�@�C������ǂݍ���		(main.cpp��main()����extern�ϐ��Ƃ��Ďg�p)
GLuint texId;		//�e�N�X�`���}�b�s���O�pID




/************************************************************
					�v���g�^�C�v�̐錾
************************************************************/
void Change_BG (data_struct *dataStr);

//***********************************�@openGL�p�̃R�[���o�b�N�֐��̐ݒ� *************************************************************
/********************************************************
�֐����FDisplay
�����@�F�`�悷�邽�߂̃R�[���o�b�N�֐�

********************************************************/
void Display(){

	static int ti=0;	//����
	//extern int t;	//����
	double time;

	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&end);
	
	//printf("%lf\n",(double)(end.QuadPart - start.QuadPart) / freq.QuadPart);
	QueryPerformanceCounter(&start);
    // Clear frame buffer
	glClearColor(0, 0, 0, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	GLfloat colors[][3] = { { 1.0f, 0.0f, 0.0f },{ 1.0f, 1.0f, 0.0f },{ 0.0f, 0.0f, 0.0f } };//red, yellow, black
	if (dataStr.outputs[0]>0.5 && dataStr.perflag == true)
	{
		glClearColor(colors[0][0], colors[0][1], colors[0][2], 1.0f); //red if NN o/p and perflag are true

	}
	else if (dataStr.outputs[0] <= 0.5 && dataStr.perflag == false)
	{
		glClearColor(colors[2][0], colors[2][1], colors[2][2], 1.0f); //black if NN o/p and perflag are false

	}
	else
	{
		glClearColor(colors[1][0], colors[1][1], colors[1][2], 1.0f); //yellow if either are true
	}

	glutPostRedisplay();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
	//check if colors work like below
	////simi neural network
	//max = 0.0;
	//for (int i = 0; i<NN_OUTPUTS; i++)
	//{
	//	if (max<dataStr.outputs[i]) {
	//		max = dataStr.outputs[i];
	//		max_num = i;
	//		
	//	}
	//	//printf("Outputs: %lf\t", dataStr.outputs[i]);
	//}
	////printf("\nMax_output: %d\t%lf\n", max_num, max);
	//// Red-Asc Failure, Green-Desc Failure, Blue-Sitting Failure, Black-Safe //simi neural network
	////change if op is 4 neurons
	////GLfloat colors[][3] = { { 1.0f, 0.0f, 0.0f },{ 1.0f, 0.0f, 1.0f },{ 0.0f, 0.0f, 1.0f },{ 0.0f, 0.0f, 0.0f } };
	//// Red- Failure, Black-Safe //simi neural network
	/*GLfloat colors[][3] = { { 1.0f, 0.0f, 0.0f },{ 0.0f, 0.0f, 0.0f }};

	glClearColor(colors[max_num][0], colors[max_num][1], colors[max_num][2], 1.0f);
	glutPostRedisplay();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);*/
	//neural network end here
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_NORMALIZE);

   
	SetData_ToolBar(ti, dataStr, &barParam);

   
	/* �}�`�̕`�� */
	
	//3D���W�n�̕`��
	switch(barParam.opeParam.drawMode){
	case 0:
		Draw_Joint3D(ti, &dataStr, &barParam, &texId);

		break;
	case 1:
		Draw_Joint2D(ti, DSP_WIDTH, DSP_HEIGHT, &dataStr, &barParam, &texId);
		
		break;
	case 2:
		Draw_URG(ti, DSP_WIDTH, DSP_HEIGHT, &dataStr, &texId);
		break;
	case 3:
		Draw_Graph(ti, DSP_WIDTH, DSP_HEIGHT, &dataStr);
		break;
	}
	
	//SetGraphBar_Param(&hBar, &barParam, "Torq_Pow", "Torq_Per");
	//printf("%lf\n",barParam.opeParam.angData[R][H]);

	//�c�[���o�[�̕`��
    TwDraw();

    glutSwapBuffers();

	 // Recall Display at next frame
    glutPostRedisplay();

	ti += 1;
	if(ti>=dataStr.length){
		ti=0;
		time_count++;
	}

	glPopMatrix();

	//Adjust_SamplingTime(t, time_count, data.length);	
	QueryPerformanceFrequency( &freq ); 
    QueryPerformanceCounter( &end );
	time = (double)(end.QuadPart - start.QuadPart)/(double)(freq.QuadPart);
	while( time<=DRAW_TIME ) {
		
		QueryPerformanceCounter( &end );
		time = (double)(end.QuadPart - start.QuadPart)/(double)(freq.QuadPart);			
	}
	//printf("%lf\n",(double)(end.QuadPart - start.QuadPart) / freq.QuadPart);
	QueryPerformanceCounter(&start);

	barParam.valParam.DrawFreq = 1/time;

}


/********************************************************
�֐����FReshape
�����@�F�E�B���h�E�̃T�C�Y���ς�������ɌĂяo�����R�[���o�b�N�֐�
�����@�Fint width		�E�B���h�E�̉���
		int height		�E�B���h�E�̏c��
		
�o�́@�Fint width
		int height
********************************************************/
void Reshape(int width, int height)
{
	//�E�B���h�E�̏c���C������ۑ�
	DSP_WIDTH = width;	DSP_HEIGHT = height;

	// Set OpenGL viewport and camera
	switch(barParam.opeParam.drawMode){
	case 0:
		glViewport(0, 0, width, height);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(40, (double)width/height, 1, 10);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(0,0,5, 0,0,0, 0,1,0);

		glRotated(10.0, 0.5, 0.0, 0.0);
		glTranslatef(0.0, -0.5, -1);

		break;
	case 1:
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		
		break;
	case 2:
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		break;
	case 3:
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		break;
	}
    

    //�c�[���o�[�ɐV�����E�B���h�E�̏c���C�����𑗂�
    TwWindowSize(width, height);

	
}


void keyboard(unsigned char key, int x, int y){
	extern bool loopFlag;
	extern bool recordFlag;

	switch (key){
	case 'z':
		barParam.wldParam.Zoom += 0.01;
		if(barParam.wldParam.Zoom>2.50)	barParam.wldParam.Zoom=2.50; 
		break;
	case 'Z':
		barParam.wldParam.Zoom -= 0.01;
		if(barParam.wldParam.Zoom<0.01)	barParam.wldParam.Zoom=0.01; 
		break;
	case 'R':
		InitParam_WorldBar(&barParam.wldParam);
		break;
	case 'r':
		recordFlag = true;
		barParam.valParam.recordFlag = true;
		printf("�L�^�J�n\n");
		break;
	case 'e':
		recordFlag = true;
		loopFlag = false;
		barParam.valParam.recordFlag = false;
		printf("�L�^�I��\n");
		break;
	

	case 'q':
	case 'Q':
	case '\033':	//\033�F�W�i��(=27)��ESC��ASCII�R�[�h	
	    printf("�I��\n");
		loopFlag = false;
		//exit(0);	//�v���O�����𐳏�I��������
	default:
	break;
	}
}


void specialKeyboard(int key, int x, int y){
	
	switch(key){
	case GLUT_KEY_LEFT:
		if(barParam.opeParam.drawMode==0)			barParam.opeParam.drawMode = static_cast<DrawMode>(DRAW_NUM-1);
		else										barParam.opeParam.drawMode = static_cast<DrawMode>(barParam.opeParam.drawMode-1);
		
		ChangeDrawMode(barParam.opeParam.drawMode);

		break;
	case GLUT_KEY_RIGHT:
		if(barParam.opeParam.drawMode==DRAW_NUM-1)	barParam.opeParam.drawMode = static_cast<DrawMode>(0);
		else										barParam.opeParam.drawMode = static_cast<DrawMode>(barParam.opeParam.drawMode+1);

		ChangeDrawMode(barParam.opeParam.drawMode);

		break;
	default:
		break;
	}
}


/********************************************************
�֐����FInit
�����@�F�E�B���h�E�̃T�C�Y���ς�������ɌĂяo�����R�[���o�b�N�֐�
�����@�Fint width		�E�B���h�E�̉���
		int height		�E�B���h�E�̏c��
		
�o�́@�Fint width
		int height
********************************************************/
void Init(){

	//�f�[�^�̓ǂݍ���
	ReadData(&dataStr);

	//�摜�f�[�^�̓ǂݍ���(��Q���p)
	cv::Ptr<IplImage> iplimg = cvLoadImage("board.jpg");
	cv::Mat img = cv::cvarrToMat(iplimg);
	cv::flip(img,img,0);
	cv::cvtColor(img,img,CV_BGR2RGB);
	
	glGenTextures(1, &texId);
	glBindTexture(GL_TEXTURE_2D, texId);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img.cols, img.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, img.data);


}


//***********************************�@���C���� *************************************************************
int draw_main(){

    float axis[] = { 0.0f, 0.7f, 0.0f };
    float angle = 0.0f;
	int argc = 1;
	char *argv[] = { (char*)"something" };

    //���C���E�B���h�E�̏����ݒ�
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(DSP_WIDTH, DSP_HEIGHT);
    glutCreateWindow("Demo");

    glutCreateMenu(NULL);

	GLenum err = glewInit();
	if(err != GLEW_OK){
		fprintf(stderr,"Err:%s\n", glewGetErrorString(err));
		return -1;
	}
	if(!glewIsExtensionSupported("GL_ARB_vertex_buffer_object")){
		puts("you Can't use VBO");
		return -1;
	}

	//wglSwapIntervalEXT( 0 );
	
	
    //glut�̃R�[���o�b�N�֐��̐ݒ�
    glutDisplayFunc(Display);
    glutReshapeFunc(Reshape);
	//glutKeyboardFunc(keyboard);			//�L�[�{�[�h���^�C�v���ꂽ�Ƃ��Ɏ��s�A�^�C�v���ꂽ�L�[��ASII
    atexit(Terminate);  // Called after glutMainLoop ends
	
	//�c�[���o�[�̏������ƍ쐬
	InitParam_WorldBar(&barParam.wldParam);
	CreateToolBars(&hBar, &barParam);

	wglSwapIntervalEXT( 0 );
	glutKeyboardFunc(keyboard);			//�L�[�{�[�h���^�C�v���ꂽ�Ƃ��Ɏ��s�A�^�C�v���ꂽ�L�[��ASII
	glutSpecialFunc(specialKeyboard);
	Init();

    // Store time
    barParam.wldParam.RotateTime = GetTimeMs();
    //�����̎p��
    SetQuaternionFromAxisAngle(axis, angle, barParam.wldParam.Rotation);
    SetQuaternionFromAxisAngle(axis, angle, barParam.wldParam.RotateStart);

	
	//���C�����[�v
	glutMainLoop();


	return 0;
}

//**********************************�@���C���������܂Ł@***************************************************





//**********************************�@�c�[���o�[�p�̃R�[���o�b�N�֐��@***************************************************
//  Callback function called when the 'AutoRotate' variable value of the tweak bar has changed
void TW_CALL SetAutoRotateCB(const void *value, void *clientData)
{
    (void)clientData; // unused

    barParam.wldParam.AutoRotate = *(const int *)value; // copy value to g_AutoRotate
    if( barParam.wldParam.AutoRotate!=0 ) 
    {
        // init rotation
        barParam.wldParam.RotateTime = GetTimeMs();
        barParam.wldParam.RotateStart[0] = barParam.wldParam.Rotation[0];
        barParam.wldParam.RotateStart[1] = barParam.wldParam.Rotation[1];
        barParam.wldParam.RotateStart[2] = barParam.wldParam.Rotation[2];
        barParam.wldParam.RotateStart[3] = barParam.wldParam.Rotation[3];

        // make Rotation variable read-only
        TwDefine(" TweakBar/ObjRotation readonly ");
    }
    else
        // make Rotation variable read-write
        TwDefine(" TweakBar/ObjRotation readwrite ");
}


//  Callback function called by the tweak bar to get the 'AutoRotate' value
void TW_CALL GetAutoRotateCB(void *value, void *clientData)
{
    (void)clientData; // unused
    *(int *)value = barParam.wldParam.AutoRotate; // copy g_AutoRotate to value
}


/********************************************************
�֐����FInitThread
�����@�F�N���e�B�J���Z�N�V�����̏������ƃX���b�h�̏����ݒ�
�����@�FCRITICAL_SECTION *cs					 �N���e�B�J���Z�N�V�����ɕK�v�ȍ\���́i�ҏW���͕s�j
		HANDLE *hThread				             �X���b�h�̃n���h��
		unsigned __stdcall threadFunc(void *p)	 �X���b�h���ŏ�������֐�
		
�o�́@�FHANDLE *hThread
********************************************************/
void InitThread(CRITICAL_SECTION *cs, HANDLE *hThread, unsigned __stdcall threadFunc(void *p)){

	InitializeCriticalSection(cs);/* �N���e�B�J���Z�N�V�����������I */

	*hThread = (HANDLE)_beginthreadex(NULL, 0, threadFunc, 0, CREATE_SUSPENDED, NULL);
	ResumeThread(*hThread);

}


/********************************************************
�֐����FInitThread
�����@�F�N���e�B�J���Z�N�V�����̏������ƃX���b�h�̏����ݒ�
�����@�FCRITICAL_SECTION *cs					 �N���e�B�J���Z�N�V�����ɕK�v�ȍ\���́i�ҏW���͕s�j
		HANDLE *hThread				             �X���b�h�̃n���h��
		
********************************************************/
void EndThread(CRITICAL_SECTION *cs, HANDLE *hThread){

	WaitForSingleObject(*hThread,INFINITE);
	CloseHandle(*hThread);
   
    DeleteCriticalSection(cs);/* �N���e�B�J���Z�N�V�����I��� */

}

//simi neural network, change background color based on output of network
void Change_BG (data_struct *dataStr)
{			
    GLfloat colors[][3] = { { 1.0f, 0.0f, 0.0f}, { 1.0f, 1.0f, 0.0f}, {1.0f, 1.0f, 1.0f } };//red, yellow, black
	if(dataStr->outputs[0]>0.5 && dataStr->perflag ==true)
	{
		glClearColor(colors[0][0], colors[0][1], colors[0][2], 1.0f); //red if NN o/p and perflag are true
		
	}
	else if (dataStr->outputs[0] <= 0.5 && dataStr->perflag == false)
	{
		glClearColor(colors[2][0], colors[2][1], colors[2][2], 1.0f); //black if NN o/p and perflag are false
		
	}
	else 
	{
		glClearColor(colors[1][0], colors[1][1], colors[1][2], 1.0f); //yellow if either are true
	}

	glutPostRedisplay();
	//check to see what kind of changes occur and if rest of the drawing info is lost and not redrawn
	//check if moving function contents to Display() function helps
}
