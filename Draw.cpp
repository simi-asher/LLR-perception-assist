/*****************************************************************************************************************************
******************************************************************************************************************************
ツールバーの初期設定のためのヘッダファイル


******************************************************************************************************************************
*****************************************************************************************************************************/

#include "Draw.h"



/********************************************************
関数名：Draw_Joint3D
説明　：3次元座標系でロボットの関節，障害物を描画する関数
引数　：const int t			       時間
		data_struct *data	       読み込んだデータ
		tweakParam  *barParam      ツールバーに設定する変数を格納した構造体
		GLuint texId[TEXTURE_NUM]  テクスチャハンドル
		
********************************************************/
void Draw_Joint3D(const int t, data_struct *data, tweakParam  *barParam, GLuint texId[TEXTURE_NUM]){

	CalculateWorldCoordinate(barParam);

	//3D座標系
	CreateGround3D();

	//障害物
	DrawBump(data->bump, texId[0]);

	//virtual wall
	DrawVWall(data->bump, texId[0]);
	
	//ロボットの関節
	DrawJoint(t, data->drawJoint);

	
}


/********************************************************
関数名：Draw_Joint2D
説明　：2次元座標系でロボットの関節，障害物を描画する関数
引数　：const int t			       時間
		int DSP_WIDTH			   ディスプレイの横幅
		int DSP_HEIGHT			   ディスプレイの縦幅
		data_struct *data	       読み込んだデータ
		tweakParam  *barParam      ツールバーに設定する変数を格納した構造体
		GLuint texId[TEXTURE_NUM]  テクスチャハンドル
		
********************************************************/
void Draw_Joint2D(const int t, int DSP_WIDTH, int DSP_HEIGHT, data_struct *data, tweakParam  *barParam, GLuint texId[TEXTURE_NUM]){

	glViewport(0, 0-300, DSP_WIDTH/2, DSP_HEIGHT-200);
	//2D座標系
	CreateGround2D();
	//障害物
	DrawBump(data->bump, texId[0]);	
	//virtual wall
	DrawVWall(data->bump, texId[0]);
	//ロボットの関節
	DrawJoint(t, data->drawJoint);

	DrawForceVector2D(data->drawJoint, barParam->valParam.perForce);

	//リアルタイムデータを描画（右上）
	glViewport(DSP_WIDTH/2, DSP_HEIGHT/2, DSP_WIDTH/2, DSP_HEIGHT/2);
	switch(barParam->opeParam.changeGraph[0]){
	case ANGLE_GRAPH:
		CreateCoordinate2D(&data->joint[t].Toe[R][Y],1,0,-1,1);
		break;
	case FOOT_GRAPH:
		CreateCoordinateTrajectory2D(data->joint[t].Toe[R],0,-1,1);
		break;
	case TORQ_OUT_GRAPH:
		CreateCoordinate2D(data->torq.out[R],3,0,-5,5);
		break;
	case TORQ_POW_GRAPH:
		CreateCoordinate2D(data->torq.pow[R],3,0,-50,50);
		break;
	case TORQ_PER_GRAPH:
		CreateCoordinate2D(data->torq.per[R],3,0,-50,50);
		break;
	default:
		break;
	}

	//リアルタイムデータを描画（右下）
	glViewport(DSP_WIDTH/2, 0, DSP_WIDTH/2, DSP_HEIGHT/2);
	switch(barParam->opeParam.changeGraph[1]){
	case ANGLE_GRAPH:
		CreateCoordinate2D(&data->joint[t].Toe[R][Y],1,1,-1,1);
		break;
	case FOOT_GRAPH:
		CreateCoordinate2D(&data->joint[t].Toe[R][Y],1,1,-1,1);
		break;
	case TORQ_OUT_GRAPH:
		CreateCoordinate2D(data->torq.out[R],3,1,-5,5);
		break;
	case TORQ_POW_GRAPH:
		CreateCoordinate2D(data->torq.pow[R],3,1,-50,50);
		break;
	case TORQ_PER_GRAPH:
		CreateCoordinate2D(data->torq.per[R],3,1,-50,50);
		break;
	default:
		break;
	}

}


/********************************************************
関数名：Draw_URG
説明　：2次元座標系でURGの生データ，X-Y座標データ，障害物検出データを描画する関数
引数　：const int t			       時間
		int DSP_WIDTH			   ディスプレイの横幅
		int DSP_HEIGHT			   ディスプレイの縦幅
		data_struct *data	       読み込んだデータ
		GLuint texId[TEXTURE_NUM]  テクスチャハンドル
		
********************************************************/
void Draw_URG(const int t, int DSP_WIDTH, int DSP_HEIGHT, data_struct *data, GLuint texId[TEXTURE_NUM]){
	
	//障害物検出データ
	glViewport(DSP_WIDTH/2, 0, DSP_WIDTH/2, DSP_HEIGHT/2);
	DrawProcessingData_URG();
	ModeObstacle(data->urg,1);

	//X-Y座標データ
	glViewport(DSP_WIDTH/2, DSP_HEIGHT/2, DSP_WIDTH/2, DSP_HEIGHT/2);
	DrawProcessingData_URG();
	ModeObstacle(data->urg,0);

	//生データ
	if(DSP_WIDTH<DSP_HEIGHT)	DSP_WIDTH = DSP_HEIGHT;
	else						DSP_HEIGHT = DSP_WIDTH;
	glViewport(0, 0, DSP_WIDTH/2, DSP_HEIGHT/2);
	CreateCoordinate_URG();
	ModeURG(data->urg);

}


/********************************************************
関数名：Draw_URG
説明　：2次元座標系でURGの生データ，X-Y座標データ，障害物検出データを描画する関数
引数　：const int t			       時間
		int DSP_WIDTH			   ディスプレイの横幅
		int DSP_HEIGHT			   ディスプレイの縦幅
		data_struct *data	       読み込んだデータ
		GLuint texId[TEXTURE_NUM]  テクスチャハンドル
		
********************************************************/
void Draw_Graph(const int t, int DSP_WIDTH, int DSP_HEIGHT, data_struct *data){

	glViewport(0, 0, DSP_WIDTH, DSP_HEIGHT);
	//2D座標系
	
	CreateCoordinate(data->weight,3,0,-50,50);
		
	
}


void ModeURG(struct drawURG_struct *drawData){
	double dataX;
	double dataY;

	glLineWidth(3);
	glColor3d(0.0, 1.0, 0.0);
	glBegin(GL_LINES);
	for (int i = 0;i < drawData[0].dataSize;i++) {
		dataX = drawData[i].length[X];
		dataY = -drawData[i].length[Y];
		glVertex2d(0, 0);
		glVertex2d(dataX, dataY);
	
	}
	glEnd();
}

void ModeObstacle(struct drawURG_struct *drawData, int mode){
	double dataX;
	double dataY;

	//検出点の描画
	glLineWidth(3);
	glColor3d(0.0, 1.0, 0.0);
	glBegin(GL_LINE_STRIP);

	switch(mode){
	case 0:
		for (int i = 0;i < drawData[0].dataSize;i++) {
			dataX = drawData[i].smooth[X];
			dataY = -drawData[i].smooth[Y];
			glVertex2d(dataX, dataY);
		}	
	
		break;
	case 1:
	for (int i = 0;i < drawData[0].dataBumpSize;i++) {
			dataX = drawData[i].position[X];
			dataY = drawData[i].position[Y];
			glVertex2d(dataX, dataY);
		}	
		glEnd();

		//検出点の描画
		glPointSize(5);
		glColor3d(1.0, 0.0, 0.0);
		glEnable(GL_POINT_SMOOTH);	
		glBegin(GL_POINTS);
		for (int i = 0;i < drawData[0].dataBumpSize;i++) {
			dataX = drawData[i].position[X];
			dataY = drawData[i].position[Y];
			glVertex2d(dataX, dataY);
		}	
		break;
	}
	glEnd();

}

/********************************************************
関数名：DrawBump
説明　：障害物を描画する関数
引数　：double bump[3]		障害物の距離，高さ，奥行き
		GLuint texId		テクスチャハンドル
		
********************************************************/
void DrawBump(double bump[7], GLuint texId){

	//テクスチャマッピングした障害物の描画
	glBindTexture(GL_TEXTURE_2D, texId);
	glEnable(GL_TEXTURE_2D);

	glLineWidth(1);						//線の太さの指定
	//glColor3d(0.9, 0.0, 0.0);
	glBegin(GL_QUADS);
	glTexCoord2f(0.f, 0.f);		glVertex3d(bump[DISTANCE], 0, -0.2);
	glTexCoord2f(1.f, 0.f);		glVertex3d(bump[DISTANCE], bump[HEIGHT], -0.2);
	glTexCoord2f(1.f, 1.f);		glVertex3d(bump[DISTANCE], bump[HEIGHT],  0.2);
	glTexCoord2f(0.f, 1.f);		glVertex3d(bump[DISTANCE], 0, 0.2);

	glTexCoord2f(0.f, 0.f);		glVertex3d(bump[DISTANCE], bump[HEIGHT], -0.2);
	glTexCoord2f(1.f, 0.f);		glVertex3d(bump[DISTANCE]+bump[DEPTH], bump[HEIGHT], -0.2);
	glTexCoord2f(1.f, 1.f);		glVertex3d(bump[DISTANCE]+bump[DEPTH], bump[HEIGHT],  0.2);
	glTexCoord2f(0.f, 1.f);		glVertex3d(bump[DISTANCE], bump[HEIGHT], 0.2);

	glTexCoord2f(0.f, 0.f);		glVertex3d(bump[DISTANCE], bump[HEIGHT], -0.2);
	glTexCoord2f(1.f, 0.f);		glVertex3d(bump[DISTANCE]+bump[DEPTH], bump[HEIGHT], -0.2);
	glTexCoord2f(1.f, 1.f);		glVertex3d(bump[DISTANCE]+bump[DEPTH], 0, -0.2);
	glTexCoord2f(0.f, 1.f);		glVertex3d(bump[DISTANCE], 0, -0.2);

	glTexCoord2f(0.f, 0.f);		glVertex3d(bump[DISTANCE], bump[HEIGHT], 0.2);
	glTexCoord2f(1.f, 0.f);		glVertex3d(bump[DISTANCE]+bump[DEPTH], bump[HEIGHT], 0.2);
	glTexCoord2f(1.f, 1.f);		glVertex3d(bump[DISTANCE]+bump[DEPTH], 0, 0.2);
	glTexCoord2f(0.f, 1.f);		glVertex3d(bump[DISTANCE], 0, 0.2);

	glTexCoord2f(0.f, 0.f);		glVertex3d(bump[DISTANCE]+bump[DEPTH], 0, -0.2);
	glTexCoord2f(1.f, 0.f);		glVertex3d(bump[DISTANCE]+bump[DEPTH], bump[HEIGHT], -0.2);
	glTexCoord2f(1.f, 1.f);		glVertex3d(bump[DISTANCE]+bump[DEPTH], bump[HEIGHT],  0.2);
	glTexCoord2f(0.f, 1.f);		glVertex3d(bump[DISTANCE]+bump[DEPTH], 0, 0.2);
	glEnd();

	glDisable(GL_TEXTURE_2D);
}

void DrawVWall(double bump[7], GLuint texId) {
	glLineWidth(1);						//線の太さの指定
	glColor3d(0.0, 1.0, 0.0);//red-ish
	glBegin(GL_LINE_STRIP);

	//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data); //from internet
	////テクスチャマッピングした障害物の描画
	//glBindTexture(GL_TEXTURE_2D, texId);
	//glEnable(GL_TEXTURE_2D);

	//glLineWidth(1);						//線の太さの指定
	//glColor3d(0.9, 0.0, 0.0);
	glBegin(GL_QUADS);
	glVertex3d(bump[VIR_START_X], 0, -0.2);
	glVertex3d(bump[VIR_START_X], bump[VIR_START_Y], -0.2);
	glVertex3d(bump[VIR_START_X], bump[VIR_START_Y], 0.2);
	glVertex3d(bump[VIR_START_X], 0, 0.2);

	glVertex3d(bump[VIR_START_X], bump[VIR_START_Y], -0.2);
	glVertex3d(bump[VIR_END_X], bump[VIR_END_Y], -0.2);
	glVertex3d(bump[VIR_END_X], bump[VIR_END_Y], 0.2);
	glVertex3d(bump[VIR_START_X], bump[VIR_START_Y], 0.2);

	glVertex3d(bump[VIR_START_X], bump[VIR_START_Y], -0.2);
	glVertex3d(bump[VIR_END_X], bump[VIR_END_Y], -0.2);
	glVertex3d(bump[VIR_END_X], 0, -0.2);
	glVertex3d(bump[VIR_START_X], 0, -0.2);

	glVertex3d(bump[VIR_START_X], bump[VIR_START_Y], 0.2);
	glVertex3d(bump[VIR_END_X], bump[VIR_END_Y], 0.2);
	glVertex3d(bump[VIR_END_X], 0, 0.2);
	glVertex3d(bump[VIR_START_X], 0, 0.2);

	glVertex3d(bump[VIR_END_X], 0, -0.2);
	glVertex3d(bump[VIR_END_X], bump[VIR_END_Y], -0.2);
	glVertex3d(bump[VIR_END_X], bump[VIR_END_Y], 0.2);
	glVertex3d(bump[VIR_END_X], 0, 0.2);
	glEnd();

	//glDisable(GL_TEXTURE_2D);
}

/********************************************************
関数名：DrawJoint
説明　：ロボットの関節を描画する関数
引数　：draw_joint *joint		ロボットの関節
		
********************************************************/
void DrawJoint(int t, draw_joint joint){
	//姿勢の描画
	glLineWidth(1);						//線の太さの指定
	glColor3d(0.2, 0.8, 0.2);
	glBegin(GL_LINE_STRIP);
	glVertex3d(joint.Hip[L][X], joint.Hip[L][Y], joint.Hip[L][Z]);
	glVertex3d(joint.Knee[L][X], joint.Knee[L][Y], joint.Knee[L][Z]);				
	glVertex3d(joint.Ankle[L][X], joint.Ankle[L][Y], joint.Ankle[L][Z]);
	glVertex3d(joint.Toe[L][X], joint.Toe[L][Y], joint.Toe[L][Z]);	
	glVertex3d(joint.Heel[L][X], joint.Heel[L][Y], joint.Heel[L][Z]);
	glVertex3d(joint.Ankle[L][X], joint.Ankle[L][Y], joint.Ankle[L][Z]);
	glEnd();
	
	glColor3d(0.0, 1.0, 0.0);		
	glBegin(GL_LINE_STRIP);
	glVertex3d(joint.Ankle[R][X], joint.Ankle[R][Y], joint.Ankle[R][Z]);
	glVertex3d(joint.Toe[R][X], joint.Toe[R][Y], joint.Toe[R][Z]);
	glVertex3d(joint.Heel[R][X], joint.Heel[R][Y], joint.Heel[R][Z]);
	glVertex3d(joint.Ankle[R][X], joint.Ankle[R][Y], joint.Ankle[R][Z]);
	glVertex3d(joint.Knee[R][X], joint.Knee[R][Y], joint.Knee[R][Z]);				
	glVertex3d(joint.Hip[R][X], joint.Hip[R][Y], joint.Hip[R][Z]);
		
	glEnd();

	glColor3d(0.0, 1.0, 0.0);		
	glBegin(GL_LINE_STRIP);
	glVertex3d(joint.Hip[R][X], joint.Hip[R][Y], joint.Hip[R][Z]);
	glVertex3d(joint.Hip[L][X], joint.Hip[L][Y], joint.Hip[L][Z]);
	glEnd();

	glColor3d(0.0, 1.0, 0.0);		
	glBegin(GL_LINE_STRIP);
	glVertex3d( (joint.Hip[R][X]+joint.Hip[L][X])/2, (joint.Hip[R][Y]+joint.Hip[L][Y])/2, (joint.Hip[R][Z]+joint.Hip[L][Z])/2 );
	glVertex3d(joint.Body[X], joint.Body[Y], joint.Body[Z]);
	glEnd();

	//関節の描画
	glPointSize(8);
	glColor3d(0.2, 0.7, 0.2);
	glBegin(GL_POINTS);
	glVertex3d(joint.Toe[R][X], joint.Toe[R][Y], joint.Toe[R][Z]);	
	glVertex3d(joint.Heel[R][X], joint.Heel[R][Y], joint.Heel[R][Z]);	
	glVertex3d(joint.Ankle[R][X], joint.Ankle[R][Y], joint.Ankle[R][Z]);
	glVertex3d(joint.Knee[R][X], joint.Knee[R][Y], joint.Knee[R][Z]);				
	glVertex3d(joint.Hip[R][X], joint.Hip[R][Y], joint.Hip[R][Z]);

	glVertex3d(joint.Hip[L][X], joint.Hip[L][Y], joint.Hip[L][Z]);
	glVertex3d(joint.Knee[L][X], joint.Knee[L][Y], joint.Knee[L][Z]);				
	glVertex3d(joint.Ankle[L][X], joint.Ankle[L][Y], joint.Ankle[L][Z]);
	glVertex3d(joint.Toe[L][X], joint.Toe[L][Y], joint.Toe[L][Z]);
	glVertex3d(joint.Heel[L][X], joint.Heel[L][Y], joint.Heel[L][Z]);	

	glVertex3d(joint.Body[X], joint.Body[Y], joint.Body[Z]);				
	glEnd();

}


/********************************************************
関数名：CreateGround3D
説明　：3次元での地面を描画する関数

********************************************************/
void CreateGround3D(){
	GLdouble vertex[PER+1][3];
	double step;

	step = (double)5.0/PER;

	for(int i=0;i<=PER;i++){
		for(int j=0;j<3;j++){
			vertex[i][j] = i*step-2;
		}
	}

	/* 3次元での地面の描画 */
	glColor3d(1.0, 1.0, 1.0);
	glBegin(GL_LINES);
	for (int i = 0; i<=PER ; ++i) {
		glVertex3d(vertex[i][X],0,vertex[0][Z]);
		glVertex3d(vertex[i][X],0,vertex[PER][Z]);
	}
    glEnd();
	glBegin(GL_LINES);
	for (int i = 0; i<=PER ; ++i) {
		glVertex3d(vertex[0][X],0,vertex[i][Z]);
		glVertex3d(vertex[PER][X],0,vertex[i][Z]);
	}
    glEnd();
}


void DrawForceVector2D(draw_joint joint, double force[3]){
	double vector[3];
	double triangleBottom;
	double triangleTop;
	double norm = 0;
	double aaa[2];
	double bbb[2];

	for(int i=0;i<3;i++)	vector[i] = force[i]/500;

	for(int i=0;i<2;i++)	norm += force[i]*force[i];
	norm = sqrt(norm);
	triangleTop = norm/1000;		triangleBottom = norm/2500;
	aaa[X] = (force[Y]*force[Y])/(norm*norm)*triangleBottom;
	aaa[Y] = (force[Y]*force[X])/(norm*norm)*triangleBottom;

	bbb[X] = force[X]/(force[X]+force[Y])*triangleTop;
	bbb[Y] = force[Y]/(force[X]+force[Y])*triangleTop;

	/***	矢印の描画		***/
	//棒の描画
	glLineWidth(3);						//線の太さの指定
	glColor3d(1.0, 1.0, 0.0);
	glBegin(GL_LINE_STRIP);
	glVertex3d(joint.Toe[R][X], joint.Toe[R][Y], joint.Toe[R][Z]);
	glVertex3d(joint.Toe[R][X]+vector[X], joint.Toe[R][Y]+vector[Y], joint.Toe[R][Z]+vector[Z]);
	glEnd();

	glPushMatrix();
	//glRotatef(-20,0,0,1.0);
	//三角形の描画
	glLineWidth(1);						//線の太さの指定
	glColor3d(1.0, 1.0, 0.0);
	glBegin(GL_TRIANGLES);
	glVertex3d(joint.Toe[R][X]+vector[X]-aaa[X], joint.Toe[R][Y]+vector[Y]+aaa[Y], joint.Toe[R][Z]);
	glVertex3d(joint.Toe[R][X]+vector[X]+aaa[X], joint.Toe[R][Y]+vector[Y]-aaa[Y], joint.Toe[R][Z]);
	glVertex3d(joint.Toe[R][X]+vector[X]+bbb[X], joint.Toe[R][Y]+vector[Y]+bbb[Y], joint.Toe[R][Z]);
	glEnd();
	glPopMatrix();


}


/********************************************************
関数名：CreateGround2D
説明　：2次元での地面を描画する関数

********************************************************/
void CreateGround2D(){
	GLdouble vertex[2];
	
	vertex[0] = -1;	vertex[1] = 1;

	/* 2次元での地面の描画 */
	glColor3d(1.0, 1.0, 1.0);
	glBegin(GL_LINES);
	glVertex3d(vertex[0],0,0);
	glVertex3d(vertex[1],0,0);
    glEnd();
	
}


/********************************************************
関数名：CreateCoordinate2D
説明　：2次元で座標系を描画する関数
引数　：double data		ロボットの関節
		int maxNum      dataの個数
		int mode		(右上of右下)に描画するグラフを設定(0:右上,1:右下)
		double min		データが取り得る最小値
		double max		データが取り得る最大値

********************************************************/
void CreateCoordinate2D(double *data, int num, int mode, double min, double max){
	
	static int t_up=0;		//右上のグラフの時間
	static int t_down=0;	//右下のグラフの時間
	//static int t=0;
	

	/* 2次元で座標系を描画 */
	glLineWidth(3);
	
	//座標系のフレームの描画
	glColor3d(1.0, 1.0, 1.0);
	glBegin(GL_LINE_LOOP);
	glVertex3d(COORDINATE2D_LEFT , COORDINATE2D_BOTTOM, 0);
	glVertex3d(COORDINATE2D_RIGHT, COORDINATE2D_BOTTOM, 0);
	glVertex3d(COORDINATE2D_RIGHT, COORDINATE2D_TOP,    0);
	glVertex3d(COORDINATE2D_LEFT , COORDINATE2D_TOP,    0);
    glEnd();

	//座標系の中線の描画
	glBegin(GL_LINES);
	glVertex3d(COORDINATE2D_LEFT , 0, 0);
	glVertex3d(COORDINATE2D_RIGHT, 0, 0);
	glEnd();

	//座標系の補助線の描画（破線）
	glEnable(GL_LINE_STIPPLE);
	glLineStipple(3, 0xCCCC);
	glLineWidth(1);
	glBegin(GL_LINES);
	
	glVertex3d(COORDINATE2D_LEFT , COORDINATE2D_BOTTOM/2, 0);
	glVertex3d(COORDINATE2D_RIGHT, COORDINATE2D_BOTTOM/2, 0);

	glVertex3d(COORDINATE2D_LEFT , COORDINATE2D_TOP/2, 0);
	glVertex3d(COORDINATE2D_RIGHT, COORDINATE2D_TOP/2, 0);
	
	glEnd();
	glDisable(GL_LINE_STIPPLE);

	
	//座標にデータを描画する
	switch(mode){
	case 0:		
		for(int i=0;i<num;i++)	DrawDataUp2D(t_up, data, i, min, max);	
		t_up++;

		break;
	case 1:
		//DrawDataDown2D(t_down, data, 0, min, max);	
		for(int i=0;i<num;i++)	DrawDataDown2D(t_down, data, i, min, max);	
		
		t_down++;

		break;
	default:
		break;
	}

	
	//t++;
}


/********************************************************
関数名：CreateCoordinate2D
説明　：2次元で座標系を描画する関数
引数　：double data		ロボットの関節
		int maxNum      dataの個数
		int mode		(右上of右下)に描画するグラフを設定(0:右上,1:右下)
		double min		データが取り得る最小値
		double max		データが取り得る最大値

********************************************************/
void CreateCoordinate(double data[EMG_CH][RULE], int num, int mode, double min, double max){
	

	/* 2次元で座標系を描画 */
	glLineWidth(3);
	
	//座標系のフレームの描画
	glColor3d(1.0, 1.0, 1.0);
	glBegin(GL_LINE_LOOP);
	glVertex3d(COORDINATE2D_LEFT , COORDINATE2D_BOTTOM, 0);
	glVertex3d(COORDINATE2D_RIGHT, COORDINATE2D_BOTTOM, 0);
	glVertex3d(COORDINATE2D_RIGHT, COORDINATE2D_TOP,    0);
	glVertex3d(COORDINATE2D_LEFT , COORDINATE2D_TOP,    0);
    glEnd();

	//座標系の中線の描画
	glBegin(GL_LINES);
	glVertex3d(COORDINATE2D_LEFT , 0, 0);
	glVertex3d(COORDINATE2D_RIGHT, 0, 0);
	glEnd();

	//座標系の補助線の描画（破線）
	glEnable(GL_LINE_STIPPLE);
	glLineStipple(3, 0xCCCC);
	glLineWidth(1);
	glBegin(GL_LINES);
	
	glVertex3d(COORDINATE2D_LEFT , COORDINATE2D_BOTTOM/2, 0);
	glVertex3d(COORDINATE2D_RIGHT, COORDINATE2D_BOTTOM/2, 0);

	glVertex3d(COORDINATE2D_LEFT , COORDINATE2D_TOP/2, 0);
	glVertex3d(COORDINATE2D_RIGHT, COORDINATE2D_TOP/2, 0);
	
	glEnd();
	glDisable(GL_LINE_STIPPLE);

	

	DrawData2D(data, 0, -10, 10);	
	
}

/********************************************************
関数名：DrawDataUp2D
説明　：2次元で座標にデータを描画する関数(右上)
引数　：const int t			時間
		GLdouble data     	描画するデータ
		int num             dataの番号
		double min			データ幅の最小値
		double max			データ幅の最大値

********************************************************/
void DrawData2D(GLdouble data[EMG_CH][RULE], int num, double min, double max){
	
	//printf("%lf\n",data);
	//データの保存
	static GLdouble x[EMG_CH][RULE], y[EMG_CH][RULE];

	for(int i=0;i<EMG_CH;i++){
		for(int j=0;j<RULE;j++){
			x[i][j] = (i*RULE+j)*( (COORDINATE2D_RIGHT-COORDINATE2D_LEFT)/(RULE*EMG_CH) );
			y[i][j] =  0.89*data[i][j]/( (max-min)/2 );
		}
	}
	

	//データの描画
	//描画色の設定
	switch(num){
	case 0:
		glColor3d(1.0, 0.0, 0.0);
		break;
	case 1:
		glColor3d(0.0, 0.0, 1.0);
		break;
	case 2:
		glColor3d(0.0, 1.0, 0.0);
		break;
	}

	glLineWidth(1);
	glBegin(GL_LINE_STRIP);

	
	
	for(int i=0;i<EMG_CH; i++){
		for(int j=0;j<RULE; j++){
			glVertex2d(x[i][j]+COORDINATE2D_LEFT, y[i][j]);
			//printf("%+12.6lf	%+12.6lf\n",x[i][j], y[i][j]);
		}
	}
	
	glEnd();
	
	
}


/********************************************************
関数名：DrawDataUp2D
説明　：2次元で座標にデータを描画する関数(右上)
引数　：const int t			時間
		GLdouble data     	描画するデータ
		int num             dataの番号
		double min			データ幅の最小値
		double max			データ幅の最大値

********************************************************/
void DrawDataUp2D(const int t, GLdouble *data, int num, double min, double max){
	
	//printf("%lf\n",data);
	//データの保存
	static GLdouble x[COORDINATE2D_MAX+1], y[3][COORDINATE2D_MAX+1];

	//データの変換(正規化)
	if(t<COORDINATE2D_MAX){
		x[t] = t*COORDINATE2D_STEP;
		y[num][t] = 0.89*data[num]/( (max-min)/2 );	//グラフの上限時にフレームとデータが重なるのを防ぐため（0.9でフレームを描画）
	}else{
		y[num][COORDINATE2D_MAX] = 0.89*data[num]/( (max-min)/2 );	//グラフの上限時にフレームとデータが重なるのを防ぐため（0.9でフレームを描画）
	}

	//データの描画
	//描画色の設定
	switch(num){
	case 0:
		glColor3d(1.0, 0.0, 0.0);
		break;
	case 1:
		glColor3d(0.0, 0.0, 1.0);
		break;
	case 2:
		glColor3d(0.0, 1.0, 0.0);
		break;
	}

	glLineWidth(1);
	glBegin(GL_LINE_STRIP);

	//データが座標の右端に来るまで
	if(t<COORDINATE2D_MAX){
		for(int i=2;i<t; i++){
			glVertex2d(x[i-1]+COORDINATE2D_LEFT, y[num][i-1]);				
			glVertex2d(x[i]+COORDINATE2D_LEFT, y[num][i]);
		}
		glEnd();
	//データが座標の右端に来たら
	}else{
		for(int i=1;i<=COORDINATE2D_MAX;i++)		y[num][i-1] = y[num][i];
	
		for(int i=1;i<COORDINATE2D_MAX; i++){
			glVertex2d(x[i-1]+COORDINATE2D_LEFT, y[num][i-1]);				
			glVertex2d(x[i]+COORDINATE2D_LEFT, y[num][i]);
		}

		glEnd();
	}
	
}


/********************************************************
関数名：DrawDataDown2D
説明　：2次元で座標にデータを描画する関数(右下)
引数　：const int t			時間
		GLdouble data     	描画するデータ
		int num             dataの番号
		double min			データ幅の最小値
		double max			データ幅の最大値

********************************************************/
void DrawDataDown2D(const int t, GLdouble *data, int num, double min, double max){
	
	//データの保存
	static GLdouble x[COORDINATE2D_MAX+1], y[3][COORDINATE2D_MAX+1];

	//データの変換(正規化)
	if(t<COORDINATE2D_MAX){
		x[t] = t*COORDINATE2D_STEP;
		y[num][t] = 0.89*data[num]/( (max-min)/2 );	//グラフの上限時にフレームとデータが重なるのを防ぐため（0.9でフレームを描画）
	}else{
		y[num][COORDINATE2D_MAX] = 0.89*data[num]/( (max-min)/2 );	//グラフの上限時にフレームとデータが重なるのを防ぐため（0.9でフレームを描画）
		//printf("%d	%lf	%lf\n",num,y[0][COORDINATE2D_MAX],y[1][COORDINATE2D_MAX]);
	}

	//データの描画
	//描画色の設定
	switch(num){
	case 0:
		glColor3d(1.0, 0.0, 0.0);
		break;
	case 1:
		glColor3d(0.0, 0.0, 1.0);
		break;
	case 2:
		glColor3d(0.0, 1.0, 0.0);
		break;
	}

	glLineWidth(1);
	glBegin(GL_LINE_STRIP);

	//データが座標の右端に来るまで
	if(t<COORDINATE2D_MAX){
		for(int i=2;i<t; i++){
			glVertex2d(x[i-1]+COORDINATE2D_LEFT, y[num][i-1]);				
			glVertex2d(x[i]+COORDINATE2D_LEFT, y[num][i]);
		}
		glEnd();
	//データが座標の右端に来たら
	}else{
		for(int i=1;i<=COORDINATE2D_MAX;i++)		y[num][i-1] = y[num][i];
	
		for(int i=1;i<COORDINATE2D_MAX; i++){
			glVertex2d(x[i-1]+COORDINATE2D_LEFT, y[num][i-1]);				
			glVertex2d(x[i]+COORDINATE2D_LEFT, y[num][i]);
		}

		glEnd();
	}
	
}


/********************************************************
関数名：CreateCoordinate2D
説明　：2次元で座標系を描画する関数
引数　：double data[2]	描画するデータ(X-Y)
		int mode		(右上of右下)に描画するグラフを設定(0:右上,1:右下)
		double min		データが取り得る最小値
		double max		データが取り得る最大値

********************************************************/
void CreateCoordinateTrajectory2D(double data[2], int mode, double min, double max){

	static int t_up=0;
	static int t_down=0;
	//static int t=0;
	//GLdouble data;

	/* 2次元で座標系を描画 */
	glLineWidth(3);
	
	//座標系のフレームの描画
	glColor3d(1.0, 1.0, 1.0);
	glBegin(GL_LINE_LOOP);
	glVertex3d(COORDINATE2D_LEFT , COORDINATE2D_BOTTOM, 0);
	glVertex3d(COORDINATE2D_RIGHT, COORDINATE2D_BOTTOM, 0);
	glVertex3d(COORDINATE2D_RIGHT, COORDINATE2D_TOP,    0);
	glVertex3d(COORDINATE2D_LEFT , COORDINATE2D_TOP,    0);
    glEnd();

	//座標系の中線の描画
	glBegin(GL_LINES);
	glVertex3d(COORDINATE2D_LEFT , 0, 0);
	glVertex3d(COORDINATE2D_RIGHT, 0, 0);
	glEnd();

	//座標系の補助線の描画（破線）
	glEnable(GL_LINE_STIPPLE);
	glLineStipple(3, 0xCCCC);
	glLineWidth(1);
	glBegin(GL_LINES);
	
	glVertex3d(COORDINATE2D_LEFT , COORDINATE2D_BOTTOM/2, 0);
	glVertex3d(COORDINATE2D_RIGHT, COORDINATE2D_BOTTOM/2, 0);

	glVertex3d(COORDINATE2D_LEFT , COORDINATE2D_TOP/2, 0);
	glVertex3d(COORDINATE2D_RIGHT, COORDINATE2D_TOP/2, 0);
	
	glEnd();
	glDisable(GL_LINE_STIPPLE);

	//data = sin(t*0.1);
	
	switch(mode){
	case 0:
		DrawDataTrajectory2D(t_up, data, min, max);
		t_up++;
		break;
	case 1:
		//DrawDataTrajectory2D(t_down, data, min, max);
		t_down++;
		break;
	default:
		break;
	}

	
	//t++;
}


/********************************************************
関数名：DrawDataTrajectory2D
説明　：2次元で座標に軌跡データを描画する関数
引数　：const int t			時間
		GLdouble data[2]   	描画するデータ(X-Y)
		double min			データ幅の最小値
		double max			データ幅の最大値

********************************************************/
void DrawDataTrajectory2D(const int t, GLdouble data[2], double min, double max){
	
	static GLdouble x[COORDINATE2D_MAX+1], y[COORDINATE2D_MAX+1];

	//データの変換
	if(t<COORDINATE2D_MAX){
		x[t] = 0.9*data[X]/( (max-min)/2 );
		y[t] = 0.9*data[Y]/( (max-min)/2 );
	}else{
		x[COORDINATE2D_MAX] = 0.9*data[X]/( (max-min)/2 );
		y[COORDINATE2D_MAX] = 0.9*data[Y]/( (max-min)/2 );
	}

	//データの描画
	//描画色の設定
	glColor3d(1.0, 0.0, 0.0);
	//glColor3d(0.0, 1.0, 0.0);
	//glColor3d(0.0, 0.0, 1.0);

	glLineWidth(1);
	glBegin(GL_LINE_STRIP);

	if(t<COORDINATE2D_MAX){
		for(int i=2;i<t; i++){
			glVertex2d(x[i-1]+COORDINATE2D_LEFT, y[i-1]);				
			glVertex2d(x[i]+COORDINATE2D_LEFT, y[i]);
		}
		glEnd();
	}else{
		for(int i=1;i<=COORDINATE2D_MAX;i++){
			x[i-1] = x[i];
			y[i-1] = y[i];
		}
	
		for(int i=1;i<COORDINATE2D_MAX; i++){
			glVertex2d(x[i-1]+COORDINATE2D_LEFT, y[i-1]);				
			glVertex2d(x[i]+COORDINATE2D_LEFT, y[i]);
		}

		glEnd();
	}
	
}


/********************************************************
関数名：CreateCoordinate_URG
説明　：2次元でURG描画用の座標系を描画する関数

********************************************************/
void CreateCoordinate_URG(){

	static int t=0;
	GLdouble data;

	//URGの背景の描画（データの描画含む）
	DrawBackground_URG();
	
}


/********************************************************
関数名：DrawBackground_URG
説明　：2次元でURG描画用の座標系を描画する関数

********************************************************/
int DrawBackground_URG(){
	
	//基準線
	glLineWidth(1);						
	glColor3d(0.9, 0.9, 0.9);
	glBegin(GL_LINES); //描画開始
	for(int i=0; i<= 2*MapResolution; i++){
		glVertex2d(-CenterX, -CenterY+perCM*i);					//縦線
		glVertex2d(CenterX, -CenterY+perCM*i);
		glVertex2d(-CenterX+perCM*i, -CenterY);					//横線
		glVertex2d(-CenterX+perCM*i, CenterY);	
	}
	glEnd();

	glLineWidth(3);						
	glColor3d(1.0, 1.0, 1.0);
	glBegin(GL_LINE_LOOP);
	glVertex2d(-CenterX, -CenterY);					//縦線
	glVertex2d(CenterX, -CenterY);						
	glVertex2d(CenterX, CenterY);					//横線
	glVertex2d(-CenterX, CenterY);	
	glEnd();

	glLineWidth(2);
	glBegin(GL_LINES);
	glVertex2d(-CenterX-0.05, 0);					//縦線
	glVertex2d(CenterX+0.05, 0);						
	glVertex2d(0, -CenterY-0.05);					//横線
	glVertex2d(0, CenterY+0.05);	
	glEnd();

	glColor3f(1.0, 0.0, 0.0);
	Circle(CenterY/2, 0, 0);


	glColor3d(0.0, 1.0, 0.0);
	DrawString("URG[fps]",10,10,-0.8,-0.8);

	return 0;
}


/********************************************************
関数名：DrawProcessingData_URG
説明　：2次元でURGの生データを処理したデータを描画する関数

********************************************************/
int DrawProcessingData_URG(){
	
	//基準線
	glLineWidth(1);						
	glColor3d(0.9, 0.9, 0.9);
	glBegin(GL_LINES); //描画開始
	for(int i=0; i<= 2*MapResolution; i++){
		glVertex2d(-CenterX, -CenterY+perCM*i);					//縦線
		glVertex2d(CenterX, -CenterY+perCM*i);
		glVertex2d(-CenterX+perCM*i, -CenterY);					//横線
		glVertex2d(-CenterX+perCM*i, CenterY);	
	}
	glEnd();

	glLineWidth(3);						
	glColor3d(1.0, 1.0, 1.0);
	glBegin(GL_LINE_LOOP);
	glVertex2d(-CenterX, -CenterY);					//縦線
	glVertex2d(CenterX, -CenterY);						
	glVertex2d(CenterX, CenterY);					//横線
	glVertex2d(-CenterX, CenterY);	
	glEnd();

	glLineWidth(2);
	glBegin(GL_LINES);
	glVertex2d(-CenterX-0.05, 0);					//縦線
	glVertex2d(CenterX+0.05, 0);						
	glVertex2d(0, -CenterY-0.05);					//横線
	glVertex2d(0, CenterY+0.05);	
	glEnd();

	
	return 0;
}


/********************************************************
関数名：Circle
説明　：2次元で丸を描画する関数
引数　：double r			円の半径
		int x			  	X方向に平行移動
		int y				Y方向に平行移動
		
********************************************************/
void Circle(double r, int x, int y){
	double rate;
	double X,Y;
	const double M_PI = 3.141592;

	glBegin(GL_LINE_LOOP); 
	for (int i = 0; i < CircleResolution; i++) {
		// 座標を計算
		rate = (double)i/CircleResolution;
		X = x + r * cos(2.0 * M_PI * rate);
		Y = y + r * sin(2.0 * M_PI * rate);
		glVertex2d(X, Y); // 頂点座標を指定
	}

	glEnd();
}



/********************************************************
関数名：DrawString
説明　：2次元で文字を描画する関数
引数　：(std::string str		円の半径
		int w					幅
		int h					高さ
		double x0			  	X方向に平行移動
		double y0				Y方向に平行移動
		
********************************************************/
void DrawString(std::string str, int w, int h, double x0, double y0) {
	glDisable(GL_LIGHTING);

	// 画面上にテキスト描画
	glRasterPos2f(x0, y0);
	int size = (int)str.size();
	for (int i = 0; i < size; ++i) {
		char ic = str[i];
		glutBitmapCharacter(GLUT_BITMAP_9_BY_15, ic);
	}
}