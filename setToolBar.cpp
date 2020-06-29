/*****************************************************************************************************************************
******************************************************************************************************************************
ツールバーの初期設定のためのソースファイル

******************************************************************************************************************************
*****************************************************************************************************************************/

#include "setToolBar.h"



/********************************************************
関数名：CreateToolBars
説明　：ツールバーの作成に関する関数
引数　：tweakHandle *hBar			ツールバーのハンドルを格納した構造体
		tweakParam  *barParam		ツールバーに設定する変数を格納した構造体
		
出力　：tweakHandle *hBar
		tweakParam  *barParam
********************************************************/
void CreateToolBars(tweakHandle *hBar, tweakParam *barParam){

	//AntTweakBarの初期化
    TwInit(TW_OPENGL, NULL);

    //GLUTのコールバック関数をセット    
    glutMouseFunc((GLUTmousebuttonfun)TwEventMouseButtonGLUT);// - Directly redirect GLUT mouse button events to AntTweakBar   
    glutMotionFunc((GLUTmousemotionfun)TwEventMouseMotionGLUT);// - Directly redirect GLUT mouse motion events to AntTweakBar    
    glutPassiveMotionFunc((GLUTmousemotionfun)TwEventMouseMotionGLUT);// - Directly redirect GLUT mouse "passive" motion events to AntTweakBar (same as MouseMotion)    
    glutKeyboardFunc((GLUTkeyboardfun)TwEventKeyboardGLUT);// - Directly redirect GLUT key events to AntTweakBar    
    glutSpecialFunc((GLUTspecialfun)TwEventSpecialGLUT);// - Directly redirect GLUT special key events to AntTweakBar
   
	// - Send 'glutGetModifers' function pointer to AntTweakBar;
    //   required because the GLUT key event functions do not report key modifiers states.
    TwGLUTModifiersFunc(glutGetModifiers);

	//各ツールバーの作成
	//絶対座標系の設定用
    hBar->worldBar = TwNewBar("WORLD");
    TwDefine(" WORLD size='200 400' color='100 150 150' position='5 20'"); // change default tweak bar size and color

	//描画モード変更用
    hBar->operationBar = TwNewBar("OPERATION");
    TwDefine(" OPERATION size='200 400' color='150 100 100' position='210 20' refresh=0.05"); // change default tweak bar size and color

	//値の表示用
    hBar->valueBar = TwNewBar("VALUE");
    TwDefine(" VALUE size='420 400' color='150 150 100' position='415 20' refresh=0.05"); // change default tweak bar size and color

	hBar->graphUpBar = TwNewBar("GRAPH_UP");
    TwDefine(" GRAPH_UP size='150 150' color='100 50 50' position='1400 50' movable=false visible=false valueswidth=40 refresh=0.05"); // change default tweak bar size and color
	
	hBar->graphDownBar = TwNewBar("GRAPH_DOWN");
    TwDefine(" GRAPH_DOWN size='150 150' color='50 50 100' position='1400 550' movable=false visible=false valueswidth=40 refresh=0.05"); // change default tweak bar size and color

	//各ツールバーの初期設定
	SetToolBar(hBar, barParam);
}


/********************************************************
関数名：SetToolBar
説明　：ツールバーの初期設定に関する関数
引数　：tweakHandle *hBar			ツールバーのハンドルを格納した構造体
		tweakParam  *barParam		ツールバーに設定する変数を格納した構造体
		
出力　：tweakHandle *hBar
		tweakParam  *barParam
********************************************************/
void SetToolBar(tweakHandle *hBar, tweakParam *barParam){

	SetWorldBar_Param(hBar->worldBar, &barParam->wldParam);
	SetValueBar_Param(hBar->valueBar, &barParam->valParam);
	SetOperationBar_Param(hBar->operationBar, barParam);
	SetGraphBar_Param(hBar, barParam, (char*)"Torq_Pow", (char*)"Torq_Per");
}


/********************************************************
関数名：SetGraphBar_Param
説明　：ツールバーの初期設定に関する関数
引数　：tweakHandle *hBar			ツールバーのハンドルを格納した構造体
		tweakParam  *barParam		ツールバーに設定する変数を格納した構造体
		char *title_up				Graph_Upのタイトル
		char *title_down　　　　　　Graph_Downのタイトル

********************************************************/
void SetGraphBar_Param(tweakHandle *hBar, tweakParam *barParam, char *title_up, char *title_down){
	char str_up[100];
	char str_down[100];

	/*TwAddVarRO(hBar->graphUpBar, "Foot[R][X]", TW_TYPE_COLOR3F, &barParam->wldParam.Color[0], "");
	TwDefine("GRAPH_UP/Foot[R][X] precision=3 visible=false");*/

	
	if(barParam->opeParam.drawMode==MODE_2D){
		wsprintf(str_up,"GRAPH_UP label='%s' ",title_up);
		TwDefine(str_up);
		
		wsprintf(str_down,"GRAPH_DOWN label='%s' ",title_down);
		TwDefine(str_down);
	}else{
		TwDefine(" GRAPH_UP    visible=false ");
		TwDefine(" GRAPH_DOWN  visible=false ");
	}

}


/********************************************************
関数名：SetWorldBar_Param
説明　：絶対座標系用ツールバーの初期設定に関する関数
引数　：TwBar *bar			ツールバーのハンドル
		worldParam *param	ツールバーに設定する変数
		
出力　：TwBar *bar
		worldParam *param
********************************************************/
void SetWorldBar_Param(TwBar *bar, worldParam *param){

	//ズーム
    TwAddVarRW(bar, "Zoom", TW_TYPE_FLOAT, &param->Zoom, 
               " min=0.01 max=2.5 step=0.01 keyIncr=z keyDecr=Z help='Scale the object (1=original size).' ");

    //回転
    TwAddVarRW(bar, "ObjRotation", TW_TYPE_QUAT4F, &param->Rotation, 
               " label='Object rotation' opened=true help='Change the object orientation.' ");

    //自動回転
    TwAddVarCB(bar, "AutoRotate", TW_TYPE_BOOL32, SetAutoRotateCB, GetAutoRotateCB, NULL, 
               " label='Auto-rotate' key=space help='Toggle auto-rotate mode.' ");

    //光源の明るさ
    TwAddVarRW(bar, "Multiplier", TW_TYPE_FLOAT, &param->LightMultiplier, 
               " label='Light booster' min=0.1 max=4 step=0.02 keyIncr='+' keyDecr='-' help='Increase/decrease the light power.' ");

    //光源の明るさ
    TwAddVarRW(bar, "LightDir", TW_TYPE_DIR3F, &param->LightDirection, 
               " label='Light direction' opened=true help='Change the light direction.' ");

    //オブジェクトの色
    TwAddVarRW(bar, "Ambient", TW_TYPE_COLOR3F, &param->MatAmbient, " group='Material' ");

    // Add 'g_MatDiffuse' to 'bar': this is a variable of type TW_TYPE_COLOR3F (3 floats color, alpha is ignored)
    // and is inserted into group 'Material'.
    TwAddVarRW(bar, "Diffuse", TW_TYPE_COLOR3F, &param->MatDiffuse, " group='Material' ");

	
}


/********************************************************
関数名：SetOperationBar_Param
説明　：描画モード変更用ツールバーの初期設定に関する関数
引数　：TwBar *bar				ツールバーのハンドル
		operationParam *param	ツールバーに設定する変数
		
出力　：TwBar *bar
		operationParam *param
********************************************************/
void SetOperationBar_Param(TwBar *bar, tweakParam *param){

	//描画モードの変更
    {
        //ツールバーに表示する描画モードの名前の設定
		TwEnumVal drawModeEV[DRAW_NUM] = { {MODE_3D, "3D"}, {MODE_2D, "2D"}, {MODE_URG, "URG"}, {MODE_GRAPH, "GRAPH"} };
        //描画モードの個数を設定
        TwType drawType = TwDefineEnum("DrawType", drawModeEV, DRAW_NUM);
        //ツールバーで描画モードを変更できるようにする
		//キーボード操作は [<] or [>]
		TwAddVarCB(bar, "DrawMode", drawType, SetDrawModeCB, GetDrawModeCB, &param->opeParam.drawMode, " keyIncr='<' keyDecr='>' help='Change draw mode.' ");
    }
	TwAddSeparator(bar, NULL, NULL);

	//
	{
		param->opeParam.changeGraph[0] = TORQ_POW_GRAPH;	param->opeParam.changeGraph[1] = TORQ_PER_GRAPH;


        //ツールバーに表示する描画モードの名前の設定
        TwEnumVal changeGraphEV[CHANGE_GRAPH_NUM] = { {NULL_GRAPH, " -"}, {ANGLE_GRAPH, "ANGLE"}, {FOOT_GRAPH, "FOOT"}, 
												      {TORQ_OUT_GRAPH, "TORQ_OUT"}, {TORQ_POW_GRAPH, "TORQ_POW"}, {TORQ_PER_GRAPH, "TORQ_PER"} };
        //描画モードの個数を設定
        TwType changeGraphType = TwDefineEnum("ChangeGraph", changeGraphEV, CHANGE_GRAPH_NUM);
        //ツールバーで描画モードを変更できるようにする
		TwAddVarCB(bar, "Up_Graph",   changeGraphType, SetChangeGraphUpCB,   GetChangeGraphCB, &param->opeParam.changeGraph[0], "group=RealTimeData");
		TwAddVarCB(bar, "Down_Graph", changeGraphType, SetChangeGraphDownCB, GetChangeGraphCB, &param->opeParam.changeGraph[1], "group=RealTimeData");
    }
	TwDefine(" OPERATION/RealTimeData  opened=false ");
	TwAddSeparator(bar, NULL, NULL);
	
	//Ngraphでの描画の変更
    {
        //Ngraphで描画するモードの名前の設定
        TwEnumVal drawGraphEV[DRAW_GRAPH_NUM] = { {NULL_DRAW, " -"}, {ANGLE_DRAW, "ANGLE"}, {FOOT_DRAW, "FOOT"}, 
												  {TORQ_OUT_DRAW, "TORQ_OUT"}, {TORQ_POW_DRAW, "TORQ_POW"}, {TORQ_PER_DRAW, "TORQ_PER"} };
        //描画の個数を設定
        TwType drawGraph = TwDefineEnum("DrawGraph", drawGraphEV, DRAW_GRAPH_NUM);
        //記録したデータをNgraphで表示する
		TwAddVarCB(bar, "OutputGraph", drawGraph, SetDrawGraphCB, GetDrawGraphCB, &param->opeParam.drawGraph, "");
    }
	TwAddSeparator(bar, NULL, NULL);

	TwAddVarCB(bar, "RECORD", TW_TYPE_BOOL32, SetRecordDataCB, GetRecordDataCB, &param->valParam.recordFlag,"keyIncr=r keyDecr=e");
	TwAddSeparator(bar, NULL, NULL);

	TwAddButton(bar, "RESET", ResetButtonCB, &param->wldParam, "key=R");
	TwAddSeparator(bar, NULL, NULL);


	TwAddButton(bar, "QUIT", QuitButtonCB, NULL, "key=q");


	//関接角度
	TwAddVarRW(bar, "Joint[R][H]", TW_TYPE_INT32, &param->opeParam.angData[R][H], 
               " min=-30  max=130 step=1 ");
	TwAddVarRW(bar, "Joint[R][K]", TW_TYPE_INT32, &param->opeParam.angData[R][K], 
               " min=-130 max=0 step=1 ");
	TwAddVarRW(bar, "Joint[R][A]", TW_TYPE_INT32, &param->opeParam.angData[R][A], 
               " min=-45  max=20 step=1 ");

}




/********************************************************
関数名：SetValueBar_Param
説明　：値表示用ツールバーの初期設定に関する関数
引数　：TwBar *bar			ツールバーのハンドル
		valueParam *param	ツールバーに設定する変数
		
出力　：TwBar *bar
		valueParam *param
********************************************************/
void SetValueBar_Param(TwBar *bar, valueParam *param){

	/***	OTHER		***/
	TwAddVarRO(bar, "Time[sec]", TW_TYPE_UINT32, &param->time,"group=OTHER");
	TwAddVarRO(bar, "SamplingFreqency[Hz]", TW_TYPE_DOUBLE, &param->Freq,"group=OTHER precision=2");
	TwAddVarRO(bar, "DrawFreqency[Hz]", TW_TYPE_DOUBLE, &param->DrawFreq,"group=OTHER precision=2");
	TwDefine(" VALUE/OTHER  opened=true ");
	TwAddSeparator(bar, NULL, NULL);

	/***	MOTION  	***/
	//足の位置
    TwAddVarRO(bar, "Foot[R][X]", TW_TYPE_DOUBLE, &param->Foot[X]," group=Foot ");
	TwAddVarRO(bar, "Foot[R][Y]", TW_TYPE_DOUBLE, &param->Foot[Y]," group=Foot ");
	TwAddVarRO(bar, "Foot[L][X]", TW_TYPE_DOUBLE, &param->Foot[X]," group=Foot ");
	TwAddVarRO(bar, "Foot[L][Y]", TW_TYPE_DOUBLE, &param->Foot[Y]," group=Foot ");
	//CoG
	TwAddVarRO(bar, "CoG[X]", TW_TYPE_DOUBLE, &param->ZmpX,"group=MOTION");
	TwAddVarRO(bar, "CoG[Y]", TW_TYPE_DOUBLE, &param->ZmpX,"group=MOTION");
	//ZMP
	TwAddVarRO(bar, "ZmpX", TW_TYPE_DOUBLE, &param->ZmpX,"group=MOTION");
	TwDefine(" VALUE/Foot       group=MOTION  opened=false ");
	TwDefine(" VALUE/MOTION  opened=false ");
	TwAddSeparator(bar, NULL, NULL);


	/***	BUMP	***/
	//障害物
    TwAddVarRO(bar, "distance", TW_TYPE_DOUBLE, &param->Bump[DISTANCE]," group=BUMP ");
	TwAddVarRO(bar, "height", TW_TYPE_DOUBLE, &param->Bump[HEIGHT]," group=BUMP ");
	TwAddVarRO(bar, "depth", TW_TYPE_DOUBLE, &param->Bump[DEPTH]," group=BUMP ");
	TwAddSeparator(bar, NULL, NULL);
    

	/***	TORQ	***/
	//出力トルク
	TwAddVarRO(bar, "torq_out[R][H]", TW_TYPE_DOUBLE, &param->Torq.out[R][H]," group=Output ");
	TwAddVarRO(bar, "torq_out[R][K]", TW_TYPE_DOUBLE, &param->Torq.out[R][K]," group=Output ");
	TwAddVarRO(bar, "torq_out[R][A]", TW_TYPE_DOUBLE, &param->Torq.out[R][A]," group=Output ");
	TwAddVarRO(bar, "torq_out[L][H]", TW_TYPE_DOUBLE, &param->Torq.out[L][H]," group=Output ");
	TwAddVarRO(bar, "torq_out[L][K]", TW_TYPE_DOUBLE, &param->Torq.out[L][K]," group=Output ");
	TwAddVarRO(bar, "torq_out[L][A]", TW_TYPE_DOUBLE, &param->Torq.out[L][A]," group=Output ");
	//パワーアシストトルク
    TwAddVarRO(bar, "torq_pow[R][H]", TW_TYPE_DOUBLE, &param->Torq.pow[R][H]," group=Power ");
	TwAddVarRO(bar, "torq_pow[R][K]", TW_TYPE_DOUBLE, &param->Torq.pow[R][K]," group=Power ");
	TwAddVarRO(bar, "torq_pow[R][A]", TW_TYPE_DOUBLE, &param->Torq.pow[R][A]," group=Power ");
	TwAddVarRO(bar, "torq_pow[L][H]", TW_TYPE_DOUBLE, &param->Torq.pow[L][H]," group=Power ");
	TwAddVarRO(bar, "torq_pow[L][K]", TW_TYPE_DOUBLE, &param->Torq.pow[L][K]," group=Power ");
	TwAddVarRO(bar, "torq_pow[L][A]", TW_TYPE_DOUBLE, &param->Torq.pow[L][A]," group=Power ");
	//認知アシストトルク
    TwAddVarRO(bar, "torq_per[R][H]", TW_TYPE_DOUBLE, &param->Torq.per[R][H]," group=Perception ");
	TwAddVarRO(bar, "torq_per[R][K]", TW_TYPE_DOUBLE, &param->Torq.per[R][K]," group=Perception ");
	TwAddVarRO(bar, "torq_per[R][A]", TW_TYPE_DOUBLE, &param->Torq.per[R][A]," group=Perception ");
	TwAddVarRO(bar, "torq_per[L][H]", TW_TYPE_DOUBLE, &param->Torq.per[L][H]," group=Perception ");
	TwAddVarRO(bar, "torq_per[L][K]", TW_TYPE_DOUBLE, &param->Torq.per[L][K]," group=Perception ");
	TwAddVarRO(bar, "torq_per[L][A]", TW_TYPE_DOUBLE, &param->Torq.per[L][A]," group=Perception ");
	TwDefine(" VALUE/Output       group=TORQ  opened=false \n"
			 " VALUE/Power        group=TORQ  opened=false \n"
			 " VALUE/Perception   group=TORQ  opened=false "   ); 
	TwAddSeparator(bar, NULL, NULL);


	/***	POWER_ASSIST	***/
	TwAddVarRO(bar, "power[L][A]", TW_TYPE_DOUBLE, &param->Torq.pow[L][A]," group=POWER_ASSIST ");
	TwDefine(" VALUE/POWER_ASSIST  opened=false ");
	TwAddSeparator(bar, NULL, NULL);


	/***	PERCEPTION_ASSIST	***/
	TwAddVarRO(bar, "perception[L][A]", TW_TYPE_DOUBLE, &param->Torq.per[L][A]," group=PERCEPTION_ASSIST ");
	TwDefine(" VALUE/PERCEPTION_ASSIST  opened=false ");
	TwAddSeparator(bar, NULL, NULL);


	/***	URG		***/
	TwAddVarRO(bar, "urg[L][A]", TW_TYPE_DOUBLE, &param->Torq.per[L][A]," group=URG ");
	TwDefine(" VALUE/URG  opened=false ");
	TwAddSeparator(bar, NULL, NULL);
	

	
 
}


/********************************************************
関数名：InitParam_WorldBar
説明　：絶対座標系の初期設定
引数　：worldParam *param	ツールバーに設定する変数
		
出力　：worldParam *param
********************************************************/
void InitParam_WorldBar(worldParam *param){
	param->Zoom = 1.0f;
	param->AutoRotate = 0;
	param->RotateTime = 0;
	param->LightMultiplier = 1.0f;

	for(int i=0;i<4;i++){
		param->Rotation[i]    = 0.0f;
		param->RotateStart[i] = 0.0f; 
		param->MatAmbient[i]  = 1.0f; 
		param->MatDiffuse[i]  = 1.0f;

		param->Color[0][i]    = 0.0f;
		param->Color[1][i]    = 0.0f;
		param->Color[2][i]    = 0.0f;
	}
	param->Rotation[3]    = 1.0f;
	param->RotateStart[3] = 1.0f;

	for(int i=0;i<3;i++)	param->Color[i][i] = 1.0f;

	for(int i=0;i<3;i++){
		param->LightDirection[i] = -0.57735f;
	}
}









/***************************************************************************************************************************************************************************************/

// Routine to set a quaternion from a rotation axis and angle
// ( input axis = float[3] angle = float  output: quat = float[4] )
void SetQuaternionFromAxisAngle(const float *axis, float angle, float *quat)
{
    float sina2, norm;
    sina2 = (float)sin(0.5f * angle);
    norm = (float)sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
    quat[0] = sina2 * axis[0] / norm;
    quat[1] = sina2 * axis[1] / norm;
    quat[2] = sina2 * axis[2] / norm;
    quat[3] = (float)cos(0.5f * angle);
}



// Routine to convert a quaternion to a 4x4 matrix
// ( input: quat = float[4]  output: mat = float[4*4] )
void ConvertQuaternionToMatrix(const float *quat, float *mat)
{
    float yy2 = 2.0f * quat[1] * quat[1];
    float xy2 = 2.0f * quat[0] * quat[1];
    float xz2 = 2.0f * quat[0] * quat[2];
    float yz2 = 2.0f * quat[1] * quat[2];
    float zz2 = 2.0f * quat[2] * quat[2];
    float wz2 = 2.0f * quat[3] * quat[2];
    float wy2 = 2.0f * quat[3] * quat[1];
    float wx2 = 2.0f * quat[3] * quat[0];
    float xx2 = 2.0f * quat[0] * quat[0];
    mat[0*4+0] = - yy2 - zz2 + 1.0f;
    mat[0*4+1] = xy2 + wz2;
    mat[0*4+2] = xz2 - wy2;
    mat[0*4+3] = 0;
    mat[1*4+0] = xy2 - wz2;
    mat[1*4+1] = - xx2 - zz2 + 1.0f;
    mat[1*4+2] = yz2 + wx2;
    mat[1*4+3] = 0;
    mat[2*4+0] = xz2 + wy2;
    mat[2*4+1] = yz2 - wx2;
    mat[2*4+2] = - xx2 - yy2 + 1.0f;
    mat[2*4+3] = 0;
    mat[3*4+0] = mat[3*4+1] = mat[3*4+2] = 0;
    mat[3*4+3] = 1;
}



// Routine to multiply 2 quaternions (ie, compose rotations)
// ( input q1 = float[4] q2 = float[4]  output: qout = float[4] )
void MultiplyQuaternions(const float *q1, const float *q2, float *qout)
{
    float qr[4];
	qr[0] = q1[3]*q2[0] + q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1];
	qr[1] = q1[3]*q2[1] + q1[1]*q2[3] + q1[2]*q2[0] - q1[0]*q2[2];
	qr[2] = q1[3]*q2[2] + q1[2]*q2[3] + q1[0]*q2[1] - q1[1]*q2[0];
	qr[3]  = q1[3]*q2[3] - (q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2]);
    qout[0] = qr[0]; qout[1] = qr[1]; qout[2] = qr[2]; qout[3] = qr[3];
}


// Return elapsed time in milliseconds
int GetTimeMs()
{
#if !defined(_WIN32)
    return glutGet(GLUT_ELAPSED_TIME);
#else
    // glutGet(GLUT_ELAPSED_TIME) seems buggy on Windows
    return (int)GetTickCount(); 
#endif
}


// Function called at exit
void Terminate(void)
{ 
    
    TwTerminate();
}


void ChangeDrawMode(DrawMode mode){
	extern double DSP_WIDTH, DSP_HEIGHT;

	switch( mode ){
	case MODE_3D:
		glViewport(0, 0, DSP_WIDTH, DSP_HEIGHT);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(40, (double)DSP_WIDTH/DSP_HEIGHT, 1, 10);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(0,0,5, 0,0,0, 0,1,0);
		glRotated(10.0, 0.5, 0.0, 0.0);
		glTranslatef(0.0, -0.5, -1);
		
		TwDefine(" GRAPH_UP    visible=false ");
		TwDefine(" GRAPH_DOWN  visible=false ");
		break;
	case MODE_2D:
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		TwDefine(" GRAPH_UP    visible=true ");
		TwDefine(" GRAPH_DOWN  visible=true ");
		
		break;
	case MODE_URG:
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		
		TwDefine(" GRAPH_UP    visible=false ");
		TwDefine(" GRAPH_DOWN  visible=false ");
		break;

	case MODE_GRAPH:
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		
		TwDefine(" GRAPH_UP    visible=false ");
		TwDefine(" GRAPH_DOWN  visible=false ");
		break;

	default:
		break;
	}
}

void TW_CALL SetDrawModeCB(const void *value, void *clientData){
    
	extern double DSP_WIDTH, DSP_HEIGHT;

	*((DrawMode *)clientData) = *(DrawMode *)value;
	DrawMode mode = *(DrawMode *)clientData; 

	ChangeDrawMode(mode);
}

void TW_CALL GetDrawModeCB(void *value, void *clientData){
    
	(void)clientData;
	*(DrawMode *)value = *((DrawMode *)clientData);
    
}


void TW_CALL SetChangeGraphUpCB(const void *value, void *clientData){

	*((ChangeGraph *)clientData) = *(ChangeGraph *)value;
	ChangeGraph mode = *(ChangeGraph *)clientData; 

	switch( mode ){
	case ANGLE_GRAPH:
		TwDefine("GRAPH_UP label='Angle' ");
		break;
	case FOOT_GRAPH:
		TwDefine("GRAPH_UP label='Foot' ");
		break;
	case TORQ_OUT_GRAPH:
		TwDefine("GRAPH_UP label='Torq_Out' ");
		break;
	case TORQ_POW_GRAPH:
		TwDefine("GRAPH_UP label='Torq_Pow' ");
		break;
	case TORQ_PER_GRAPH:
		TwDefine("GRAPH_UP label='Torq_Per' ");
		break;

	default:
		break;
	}
}

void TW_CALL SetChangeGraphDownCB(const void *value, void *clientData){

	*((ChangeGraph *)clientData) = *(ChangeGraph *)value;
	ChangeGraph mode = *(ChangeGraph *)clientData; 

	switch( mode ){
	case ANGLE_GRAPH:
		TwDefine("GRAPH_DOWN label='Angle' ");
		break;
	case FOOT_GRAPH:
		TwDefine("GRAPH_DOWN label='Foot' ");
		break;
	case TORQ_OUT_GRAPH:
		TwDefine("GRAPH_DOWN label='Torq_Out' ");
		break;
	case TORQ_POW_GRAPH:
		TwDefine("GRAPH_DOWN label='Torq_Pow' ");
		break;
	case TORQ_PER_GRAPH:
		TwDefine("GRAPH_DOWN label='Torq_Per' ");
		break;

	default:
		break;
	}
}

void TW_CALL GetChangeGraphCB(void *value, void *clientData){
    
	(void)clientData;
	*(ChangeGraph *)value = *((ChangeGraph *)clientData);
    
}


void TW_CALL SetDrawGraphCB(const void *value, void *clientData){
    
	*((DrawGraph *)clientData) = *(DrawGraph *)value;
	DrawGraph mode = *(DrawGraph *)clientData; 

	switch( mode ){
	case ANGLE_DRAW:
		system("output_data\\Ngraph.ngp");
		break;
	case FOOT_DRAW:
		system("output_data\\Position\\TRAJECTORY_FOOT[R].ngp");
		break;
	case TORQ_OUT_DRAW:
		system("output_data\\Torq\\OUTPUT.ngp");
		break;
	case TORQ_POW_DRAW:
		system("output_data\\Torq\\POWER_ASSIST.ngp");
		break;
	case TORQ_PER_DRAW:
		system("output_data\\Torq\\PERCEPTION_ASSIST.ngp");
		break;

	default:
		break;
	}
}

void TW_CALL GetDrawGraphCB(void *value, void *clientData){
    
	(void)clientData;
	*(DrawGraph *)value = *((DrawGraph *)clientData);
    
}


void TW_CALL SetRecordDataCB(const void *value, void *clientData){
    
	*((valueParam *)clientData) = *(valueParam *)value;

}


//  Callback function called by the tweak bar to get the 'AutoRotate' value
void TW_CALL GetRecordDataCB(void *value, void *clientData){
    
	(void)clientData;
	*(bool *)value = *((bool *)clientData);

}



void TW_CALL ResetButtonCB(void *clientData){

	worldParam param = *(worldParam *)clientData;
	InitParam_WorldBar(&param);

	*(worldParam *)clientData = param;

}


void TW_CALL QuitButtonCB(void *clientData){

	extern bool loopFlag;
	loopFlag = false;

}

