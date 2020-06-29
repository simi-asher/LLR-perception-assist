/*****************************************************************************************************************************
******************************************************************************************************************************
�`��֌W�̌v�Z�Ɋւ���\�[�X�t�@�C��


******************************************************************************************************************************
*****************************************************************************************************************************/

#include "calculateDraw.h"



/********************************************************
�֐����FCalculateWorldCoordinate
�����@�F���W�n�̉�]�C�Y�[������ь����̐ݒ�Ɋւ���֐�
�����@�FtweakParam  *barParam		�c�[���o�[�ɐݒ肷��ϐ����i�[�����\����
		
�o�́@�FtweakParam  *barParam
********************************************************/
void CalculateWorldCoordinate(tweakParam  *barParam){
	//float v[4]; //�����̐ݒ�p�ϐ�
	float mat[4*4]; //���W�n�̉�]�Ɋւ���ϐ�

	 //�����̐ݒ�
    /*glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    v[0] = v[1] = v[2] = barParam->wldParam.LightMultiplier*0.4f; v[3] = 1.0f;
    glLightfv(GL_LIGHT0, GL_AMBIENT, v);
    v[0] = v[1] = v[2] = barParam->wldParam.LightMultiplier*0.8f; v[3] = 1.0f;
    glLightfv(GL_LIGHT0, GL_DIFFUSE, v);
    v[0] = -barParam->wldParam.LightDirection[0]; v[1] = -barParam->wldParam.LightDirection[1]; v[2] = -barParam->wldParam.LightDirection[2]; v[3] = 0.0f;
    glLightfv(GL_LIGHT0, GL_POSITION, v);*/

	//���̂ւ̌��̓��肩��
    //glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, barParam->wldParam.MatAmbient);
    //glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, barParam->wldParam.MatDiffuse);

    //���W�n�̉�]
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

	//�Y�[��
    glScalef(barParam->wldParam.Zoom, barParam->wldParam.Zoom, barParam->wldParam.Zoom);

}



