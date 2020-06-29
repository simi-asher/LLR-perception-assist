/*******************************************************************************
	

		下肢パワーアシストロボットを描画するプログラム
		
		製作者　hora
	　　製作日　2016/11/25
		更新日　2016/11/25

		※環境構築　AntTweakBarのdll,lib,hファイルを所定のフォルダに入れる必要あり
					openGLの拡張機能としてglutを入れる必要あり
********************************************************************************/

#include "mainDraw.h"




/************************************************************
					グローバル変数
************************************************************/
double DSP_WIDTH  = 1200;	//ウィンドウの横幅
double DSP_HEIGHT = 1920;   //ウィンドウの縦幅

int time_count = 0; //時間がマックス値になった回数
double max = 0.0;
int max_num = 0;
int drawMode=0;
LARGE_INTEGER freq,start,end;

/************************************************************
					ツールバー用の変数
************************************************************/
tweakHandle hBar;		//ツールバーのハンドルを格納するための構造体
tweakParam  barParam;	//ツールバーのパラメータを格納するための構造体	(main.cppのmain()内でextern変数として使用)

/************************************************************
					データ格納用の変数
************************************************************/
data_struct dataStr;	//datファイルから読み込み		(main.cppのmain()内でextern変数として使用)
GLuint texId;		//テクスチャマッピング用ID




/************************************************************
					プロトタイプの宣言
************************************************************/
void Change_BG (data_struct *dataStr);

//***********************************　openGL用のコールバック関数の設定 *************************************************************
/********************************************************
関数名：Display
説明　：描画するためのコールバック関数

********************************************************/
void Display(){

	static int ti=0;	//時間
	//extern int t;	//時間
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

   
	/* 図形の描画 */
	
	//3D座標系の描画
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

	//ツールバーの描画
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
関数名：Reshape
説明　：ウィンドウのサイズが変わった時に呼び出されるコールバック関数
引数　：int width		ウィンドウの横幅
		int height		ウィンドウの縦幅
		
出力　：int width
		int height
********************************************************/
void Reshape(int width, int height)
{
	//ウィンドウの縦幅，横幅を保存
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
    

    //ツールバーに新しいウィンドウの縦幅，横幅を送る
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
		printf("記録開始\n");
		break;
	case 'e':
		recordFlag = true;
		loopFlag = false;
		barParam.valParam.recordFlag = false;
		printf("記録終了\n");
		break;
	

	case 'q':
	case 'Q':
	case '\033':	//\033：８進数(=27)はESCのASCIIコード	
	    printf("終了\n");
		loopFlag = false;
		//exit(0);	//プログラムを正常終了させる
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
関数名：Init
説明　：ウィンドウのサイズが変わった時に呼び出されるコールバック関数
引数　：int width		ウィンドウの横幅
		int height		ウィンドウの縦幅
		
出力　：int width
		int height
********************************************************/
void Init(){

	//データの読み込み
	ReadData(&dataStr);

	//画像データの読み込み(障害物用)
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


//***********************************　メイン文 *************************************************************
int draw_main(){

    float axis[] = { 0.0f, 0.7f, 0.0f };
    float angle = 0.0f;
	int argc = 1;
	char *argv[] = { (char*)"something" };

    //メインウィンドウの初期設定
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
	
	
    //glutのコールバック関数の設定
    glutDisplayFunc(Display);
    glutReshapeFunc(Reshape);
	//glutKeyboardFunc(keyboard);			//キーボードがタイプされたときに実行、タイプされたキーのASII
    atexit(Terminate);  // Called after glutMainLoop ends
	
	//ツールバーの初期化と作成
	InitParam_WorldBar(&barParam.wldParam);
	CreateToolBars(&hBar, &barParam);

	wglSwapIntervalEXT( 0 );
	glutKeyboardFunc(keyboard);			//キーボードがタイプされたときに実行、タイプされたキーのASII
	glutSpecialFunc(specialKeyboard);
	Init();

    // Store time
    barParam.wldParam.RotateTime = GetTimeMs();
    //初期の姿勢
    SetQuaternionFromAxisAngle(axis, angle, barParam.wldParam.Rotation);
    SetQuaternionFromAxisAngle(axis, angle, barParam.wldParam.RotateStart);

	
	//メインループ
	glutMainLoop();


	return 0;
}

//**********************************　メイン文ここまで　***************************************************





//**********************************　ツールバー用のコールバック関数　***************************************************
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
関数名：InitThread
説明　：クリティカルセクションの初期化とスレッドの初期設定
引数　：CRITICAL_SECTION *cs					 クリティカルセクションに必要な構造体（編集等は不可）
		HANDLE *hThread				             スレッドのハンドル
		unsigned __stdcall threadFunc(void *p)	 スレッド内で処理する関数
		
出力　：HANDLE *hThread
********************************************************/
void InitThread(CRITICAL_SECTION *cs, HANDLE *hThread, unsigned __stdcall threadFunc(void *p)){

	InitializeCriticalSection(cs);/* クリティカルセクション初期化！ */

	*hThread = (HANDLE)_beginthreadex(NULL, 0, threadFunc, 0, CREATE_SUSPENDED, NULL);
	ResumeThread(*hThread);

}


/********************************************************
関数名：InitThread
説明　：クリティカルセクションの初期化とスレッドの初期設定
引数　：CRITICAL_SECTION *cs					 クリティカルセクションに必要な構造体（編集等は不可）
		HANDLE *hThread				             スレッドのハンドル
		
********************************************************/
void EndThread(CRITICAL_SECTION *cs, HANDLE *hThread){

	WaitForSingleObject(*hThread,INFINITE);
	CloseHandle(*hThread);
   
    DeleteCriticalSection(cs);/* クリティカルセクション終わり */

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
