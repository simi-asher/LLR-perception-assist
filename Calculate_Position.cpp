/*****************************************************************************************************************************
******************************************************************************************************************************
姿勢計算に必要な関数をまとめたソースファイル



******************************************************************************************************************************
*****************************************************************************************************************************/

#include "Calculate_Position.h"



/********************************************************
関数名：Calculate_Position
説明　：姿勢計算のための関数
		メインループ内で使用
引数　：double enc_data[2][3]	      関節角度
		struct leg_state Leg          足の状態（立脚，遊脚）
		struct body_posi *body_posi	  姿勢計算のための構造体
	　　
出力　：struct leg_state Leg
		struct body_posi *body_posi
********************************************************/
void Calculate_Position(double enc_data[2][3], struct leg_state *Leg, struct body_posi *body_posi, float *Yaw1,float *Roll, float *Pitch2, float *Yaw2){ //simi MPU removed float *Pitch1,
	
	LegState(Leg->state, Leg);

	Calculation_Body_Part_Pos(AnkleBase, enc_data, Leg,	body_posi, Yaw1, Roll,  Pitch2, Yaw2); //simi MPU removed Pitch1,

}


/********************************************************
関数名：LegState
説明　：足の状態（立脚，遊脚）
引数　：int state				立脚側の関節角度
		struct leg_state Leg    足の状態（立脚，遊脚）
	　　
出力　：struct leg_state Leg
********************************************************/
void LegState( int state, struct leg_state *Leg ){
	
	switch(state){
	case Stand:
		break;
	case SwingR:
		Leg->support = L;
		Leg->swing   = R;
		break;
	case SwingL:
		Leg->support = R;
		Leg->swing   = L;
		break;
	}
}



/********************************************************
関数名：Calculation_Body_Orientation
説明　：上体の角度計算（地面基準）
引数　：double enc_data[3]				立脚側の関節角度
	　　
戻り値：enc_data[H]+enc_data[K]+enc_data[A]
********************************************************/
double Calculation_Body_Orientation( double enc_data[3] ){
	return ( enc_data[H]+enc_data[K]+enc_data[A] );
}

//simi to resolve error
void Calculation_Swing_Angle(double enc_data[2][3], leg_state * Leg, double theta[3])
{
	printf("in Calc swing angle func");
}


/********************************************************
関数名：Calculation_Swing_Angle
説明　：遊脚の関節角度計算（遊脚）
引数　：double enc_data[3]				立脚側の関節角度
		struct leg_state *Leg			足の状態（立脚，遊脚）
		double theta[3]					遊脚の関節角度（地面基準）
	　　
出力  ：double theta[3]	
********************************************************/

void Calculation_Swing_Angle(double enc_data[2][3], struct leg_state *Leg, double theta[3], double body_ang){
	
	double ang_body = 0;
	const int SUP = Leg->support;
	const int SW  = Leg->swing;

	//ang_body = Calculation_Body_Orientation(enc_data[SUP]); //simi the old way-use for sit squat
	ang_body = body_ang; //simi using MPU//comment out for sit,squat
	theta[H] = enc_data[SW][H];// -ang_body; //only for sit
	theta[K] = enc_data[SW][K];
	theta[A] = enc_data[SW][A];// +(PI / 2); //simi check

}



/********************************************************
関数名：Calculation_Body_Part_Pos
説明　：x-y座標基準で姿勢の計算
引数　：int mode						股関節ベースor足首ベース
		double enc_data[3]				立脚側の関節角度
		struct leg_state *Leg			足の状態（立脚，遊脚）
		double theta[3]					遊脚の関節角度（地面基準）
	　　
出力  ：double theta[3]	
********************************************************/
void Calculation_Body_Part_Pos(int mode, double enc_data[2][3], struct leg_state *Leg,	struct body_posi *body_posi, float *Yaw1, float *Roll, float *Pitch2, float *Yaw2){//float *Pitch1,
	
//	int swing;		//遊脚：引数にとる
	double ang_body;
	double theta[3] = {0.0};
	
	const int SUP = Leg->support; //exp
	const int SW  = Leg->swing;

	switch(mode){
	case HipBase:
		//支持脚座標計算		踵とつま先が地面に接地していると仮定
		for(int i=0;i<2;i++){
		(*body_posi).Hip[i][X]   = 0;
		(*body_posi).Hip[i][Y]   = 0;
		(*body_posi).Knee[i][X]  = (*body_posi).Hip[i][X]    + LEN_H_K*sin( enc_data[i][H] );
		(*body_posi).Knee[i][Y]  = (*body_posi).Hip[i][Y]    - LEN_H_K*cos( enc_data[i][H] );
		(*body_posi).Ankle[i][X] = (*body_posi).Knee[i][X]   + LEN_K_A*sin( enc_data[i][H]+enc_data[i][K] );		
		(*body_posi).Ankle[i][Y] = (*body_posi).Knee[i][Y]   - LEN_K_A*cos( enc_data[i][H]+enc_data[i][K] );
		(*body_posi).Sole[i][X]  = (*body_posi).Ankle[i][X]  + LEN_A_S*sin( enc_data[i][H]+enc_data[i][K]+enc_data[i][A] );		  //支持脚足首原点
		(*body_posi).Sole[i][Y]  = (*body_posi).Ankle[i][Y]  - LEN_A_S*cos( enc_data[i][H]+enc_data[i][K]+enc_data[i][A] );
		(*body_posi).Heel[i][X]  = (*body_posi).Sole[i][X]   - LEN_S_H*cos( enc_data[i][H]+enc_data[i][K]+enc_data[i][A] );		
		(*body_posi).Heel[i][Y]  = (*body_posi).Sole[i][Y]   - LEN_S_H*sin( enc_data[i][H]+enc_data[i][K]+enc_data[i][A] );
		(*body_posi).Toe[i][X]   = (*body_posi).Sole[i][X]   + LEN_S_T*cos( enc_data[i][H]+enc_data[i][K]+enc_data[i][A] );
		(*body_posi).Toe[i][Y]   = (*body_posi).Sole[i][Y]   + LEN_S_T*sin( enc_data[i][H]+enc_data[i][K]+enc_data[i][A] );

		(*body_posi).Urg[i][X]   = (*body_posi).Knee[i][X]   + LEN_K_U*cos( enc_data[i][H] );
		(*body_posi).Urg[i][Y]   = (*body_posi).Knee[i][Y]   + LEN_K_U*sin( enc_data[i][H] );
	
		}

		////接地点の計算
		//LowestPosi = LowestPos(body_posi);
		//for(int i=0;i<2;i++){
		//	(*body_posi).Toe[i][Y]	 -= LowestPosi;
		//	(*body_posi).Heel[i][Y]	 -= LowestPosi;
		//	(*body_posi).Sole[i][Y]	 -= LowestPosi;
		//	(*body_posi).Ankle[i][Y] -= LowestPosi;
		//	(*body_posi).Knee[i][Y]  -= LowestPosi;
		//	(*body_posi).Hip[i][Y]   -= LowestPosi;

		//	(*body_posi).Urg[i][Y]   -= LowestPosi;
		//}


		// body pos//
		(*body_posi).Body.pos[X]=(*body_posi).Hip[SUP][X];
		(*body_posi).Body.pos[Y]=(*body_posi).Hip[SUP][Y]+LEN_BODY;

		break;
	

	case AnkleBase: //simi MPU; not changed for hip base
		//支持脚座標計算		踵とつま先が地面に接地していると仮定
		(*body_posi).Sole[SUP][X]  = 0.0;		  //支持脚足首原点
		(*body_posi).Sole[SUP][Y]  = 0.0;
		(*body_posi).Ankle[SUP][X] = 0.0;		
		(*body_posi).Ankle[SUP][Y] = 0.0 + LEN_A_S;
		(*body_posi).Heel[SUP][X]  = 0.0 - LEN_S_H;		
		(*body_posi).Heel[SUP][Y]  = 0.0;
		(*body_posi).Toe[SUP][X]   = (*body_posi).Sole[SUP][X]  + LEN_S_T;
		(*body_posi).Toe[SUP][Y]   = (*body_posi).Sole[SUP][Y]  + 0.0;
		(*body_posi).Knee[SUP][X]  = (*body_posi).Ankle[SUP][X] + LEN_K_A*sin( enc_data[SUP][A] );
		(*body_posi).Knee[SUP][Y] = (*body_posi).Ankle[SUP][Y] + LEN_K_A * cos(enc_data[SUP][A]); // *cos(*Pitch1*DtoR); //simi MPU
		(*body_posi).Hip[SUP][X]   = (*body_posi).Knee[SUP][X]  - LEN_H_K*sin( -enc_data[SUP][K]-enc_data[SUP][A] );		//屈曲時の膝関節のエンコーダの値が負なので正になおすため (-enc_ang[SUP][K])
		(*body_posi).Hip[SUP][Y] = (*body_posi).Knee[SUP][Y] + LEN_H_K * cos(-enc_data[SUP][K] - enc_data[SUP][A]);// *cos(*Pitch1*DtoR);		//Because the value of the knee joint encoder at the time of flexion is negative, it is corrected(-enc_ang[SUP][K])
		//printf("%lf \t%lf \t%lf \n", enc_data[SUP][A], enc_data[SUP][K], enc_data[SUP][H]);
		(*body_posi).Urg[SUP][X] = (*body_posi).Knee[SUP][X] + LEN_K_U * cos(enc_data[SUP][K]+ enc_data[SUP][A]);  //simi + LEN_K_U*cos(  enc_data[SUP][A]+enc_data[SUP][K] );
		(*body_posi).Urg[SUP][Y] = (*body_posi).Knee[SUP][Y] - LEN_K_U * sin(enc_data[SUP][A] + enc_data[SUP][K]);  //simi 

		ang_body = (double)-*Roll*DtoR;
		Calculation_Swing_Angle( enc_data, Leg, theta, ang_body); //thta A has +Pi/2. why?
		//printf("%lf %lf %lf %lf \n", ang_body, theta[H], theta[K], theta[A]);

		//遊脚座標計算
		(*body_posi).Hip[SW][X] = (*body_posi).Hip[SUP][X]+LEN_LR_Hip * sin(*Yaw2*DtoR); //simi MPU
		(*body_posi).Hip[SW][Y] = (*body_posi).Hip[SUP][Y];// +LEN_LR_Hip * sin(-*Pitch2*DtoR); //simi MPU
		
		(*body_posi).Knee[SW][X]  = (*body_posi).Hip[SW][X]    + LEN_H_K*sin( theta[H] );
		(*body_posi).Knee[SW][Y]  = (*body_posi).Hip[SW][Y]    - LEN_H_K*cos( theta[H] );
		(*body_posi).Ankle[SW][X] = (*body_posi).Knee[SW][X]   + LEN_K_A*sin( theta[H]+theta[K] );					//simi check//膝関節の角度で引くため To pull at the knee joint angle (+theta[K])
		(*body_posi).Ankle[SW][Y] = (*body_posi).Knee[SW][Y]   - LEN_K_A*cos( theta[H]+theta[K] );
		
		//(*body_posi).Sole[SW][X]  = (*body_posi).Ankle[SW][X]  - LEN_A_S*cos( theta[H]+theta[K]+theta[A] );		//膝関節の角度で引くため (+theta[K]) sin(θ+1/2π) = cosθ
		//(*body_posi).Sole[SW][Y]  = (*body_posi).Ankle[SW][Y]  - LEN_A_S*sin( theta[H]+theta[K]+theta[A] );	
		//(*body_posi).Toe[SW][X]   = (*body_posi).Sole[SW][X]   + LEN_S_T*sin( theta[H]+theta[K]+theta[A] );		//膝関節の角度で引くため (+theta[K]) sin(θ+1/2π) = cosθ
		//(*body_posi).Toe[SW][Y]   = (*body_posi).Sole[SW][Y]   - LEN_S_T*cos( theta[H]+theta[K]+theta[A] );		
		//(*body_posi).Heel[SW][X] = (*body_posi).Sole[SW][X] - LEN_S_H * sin(theta[H] + theta[K] + theta[A]);		//膝関節の角度で引くため (+theta[K]) sin(θ+1/2π) = cosθ
		//(*body_posi).Heel[SW][Y] = (*body_posi).Sole[SW][Y] + LEN_S_H * cos(theta[H] + theta[K] + theta[A]);		//									 cos(θ+1/2π) = -sinθ
		//simi changes
		(*body_posi).Sole[SW][X] = (*body_posi).Ankle[SW][X] + LEN_A_S * sin(theta[H] + theta[K] + theta[A]);
		(*body_posi).Sole[SW][Y] = (*body_posi).Ankle[SW][Y] - LEN_A_S * cos(theta[H] + theta[K] + theta[A]);
		(*body_posi).Toe[SW][X] = (*body_posi).Sole[SW][X] + LEN_S_T * cos(theta[H] + theta[K] + theta[A]);		//膝関節の角度で引くため (+theta[K]) sin(θ+1/2π) = cosθ
		(*body_posi).Toe[SW][Y]   = (*body_posi).Sole[SW][Y]   + LEN_S_T*sin( theta[H]+theta[K]+theta[A] );		//cos(θ+1/2π) = -sinθ
		(*body_posi).Heel[SW][X]  = (*body_posi).Sole[SW][X]   - LEN_S_H*cos( theta[H]+theta[K]+theta[A] );		//膝関節の角度で引くため (+theta[K]) sin(θ+1/2π) = cosθ
		(*body_posi).Heel[SW][Y]  = (*body_posi).Sole[SW][Y]   - LEN_S_H*sin( theta[H]+theta[K]+theta[A] );		//									 cos(θ+1/2π) = -sinθ
	
		(*body_posi).Urg[SW][X]   = (*body_posi).Knee[SW][X]   + LEN_K_U*cos(  enc_data[SW][H] );
		(*body_posi).Urg[SW][Y]   = (*body_posi).Knee[SW][Y]   + LEN_K_U*sin(  enc_data[SW][H] );

		//simi
		//ang_body = Calculation_Body_Orientation( enc_data[SUP]);		//なんか計算重複して無駄な気がするけど、直すの面倒なんでとりあえずこれで
		//keep commented out for sit squat too and use roll to calc body pos
		// body pos//
		(*body_posi).Body.pos[X]=(*body_posi).Hip[SUP][X]+LEN_BODY*sin( ang_body );
		(*body_posi).Body.pos[Y]=(*body_posi).Hip[SUP][Y]+LEN_BODY*cos( ang_body );
		break;
	}

}


