/*****************************************************************************************************************************
******************************************************************************************************************************
描画関係の計算に関するソースファイル


******************************************************************************************************************************
*****************************************************************************************************************************/

#include "calculateDraw.h"



/********************************************************
関数名：CalculateWorldCoordinate
説明　：座標系の回転，ズームおよび光源の設定に関する関数
引数　：tweakParam  *barParam		ツールバーに設定する変数を格納した構造体
		
出力　：tweakParam  *barParam
********************************************************/
void CalculateWorldCoordinate(tweakParam  *barParam){
	//float v[4]; //光源の設定用変数
	float mat[4*4]; //座標系の回転に関する変数

	 //光源の設定
    /*glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    v[0] = v[1] = v[2] = barParam->wldParam.LightMultiplier*0.4f; v[3] = 1.0f;
    glLightfv(GL_LIGHT0, GL_AMBIENT, v);
    v[0] = v[1] = v[2] = barParam->wldParam.LightMultiplier*0.8f; v[3] = 1.0f;
    glLightfv(GL_LIGHT0, GL_DIFFUSE, v);
    v[0] = -barParam->wldParam.LightDirection[0]; v[1] = -barParam->wldParam.LightDirection[1]; v[2] = -barParam->wldParam.LightDirection[2]; v[3] = 0.0f;
    glLightfv(GL_LIGHT0, GL_POSITION, v);*/

	//物体への光の当りかた
    //glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, barParam->wldParam.MatAmbient);
    //glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, barParam->wldParam.MatDiffuse);

    //座標系の回転
    glPushMatrix();
    glTranslatef(0.5f, -0.3f, 0.0f);
    if( barParam->wldParam.AutoRotate ) 
    {
        float axis[3] = { 0, 1, 0 };
        float angle = (float)(GetTimeMs()-barParam->wldParam.RotateTime)/1000.0f;
        float quat[4];
        SetQuaternionFromAxisAngle(axis, angle, quat);
        MultiplyQuaternions(barParam->wldParam.RotateStart, quat, barParam->wldParam.Rotation);
    }
    ConvertQuaternionToMatrix(barParam->wldParam.Rotation, mat);
    glMultMatrixf(mat);

	//ズーム
    glScalef(barParam->wldParam.Zoom, barParam->wldParam.Zoom, barParam->wldParam.Zoom);

}



