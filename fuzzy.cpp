/*****************************************************************************************************************************
******************************************************************************************************************************
トルク計算に必要な関数をまとめたヘッダファイル



******************************************************************************************************************************
*****************************************************************************************************************************/

#include "fuzzy.h"
#pragma warning(disable:4996)

/********************************************************
関数名：FuzzyNeuro
説明　：ファジィニューロ調整器の計算用の関数
		メインループ内で使用
引数　：const int learnFlag			  ウエイト値を学習させるかどうかのフラグ
		_weight *weight		　		  ファジィ制御に必要なウエイト値
		double ang[3]				　関節角度
		double rms[EMG_CH]			  筋電のrms値
　　　　double force[3][3]　　　　　　力センサの値
		double weight_p[3][EMG_CH]	　関節ごとのトルク推定のためのウエイト値
		
出力　：double weight_p[3][EMG_CH]
********************************************************/
int FuzzyNeuro(const int learnFlag, _weight *weight, double ang[3], double rms[EMG_CH], double force[3][3], double weight_p[3][EMG_CH]){

	int i,j;

	double error_force[3];

	double theta[3];
	double c_weight[3][EMG_CH];		// quantity of change from init weight	要素数要チェック
	double f_layer[3][4];				// elements of fuzzifier layer
	double r_layer[RULE];				// elements of rule layer
	double d_layer[3][EMG_CH];		// elements of defuzzifier layer
	double d_layer_sum;					// one element of defuzzifier layer

	//rad→degree
	for( i=0;i<2;i++ )	theta[i] = ang[i] * RtoD;
		
	//人の動きとロボットの動きとの差を計測
	Error_Force(force, error_force);

	// calculate elements of fuzzifier layer //
	f_layer[H][PO_B] = Sig_Func( theta[H], weight->cen[H][PO_B], weight->dev[H][PO_B] );
	f_layer[H][PO_S] = Gau_Func( theta[H], weight->cen[H][PO_S], weight->dev[H][PO_S] );
	f_layer[H][ZERO] = Gau_Func( theta[H], weight->cen[H][ZERO], weight->dev[H][ZERO] );
	f_layer[H][NE_S] = Sig_Func( theta[H], weight->cen[H][NE_S], weight->dev[H][NE_S] );

	f_layer[K][PO_B] = Sig_Func( theta[K], weight->cen[K][PO_B], weight->dev[K][PO_B] );
	f_layer[K][PO_S] = Gau_Func( theta[K], weight->cen[K][PO_S], weight->dev[K][PO_S] );
	f_layer[K][ZERO] = Gau_Func( theta[K], weight->cen[K][ZERO], weight->dev[K][ZERO] );
	f_layer[K][NE_S] = Sig_Func( theta[K], weight->cen[K][NE_S], weight->dev[K][NE_S] );

	f_layer[A][0] = Sig_Func( theta[A], weight->cen[A][0], weight->dev[A][0] );
	f_layer[A][1] = Sig_Func( theta[A], weight->cen[A][1], weight->dev[A][1] );

	// calculate elements of rule layer //
	
	r_layer[0]  = f_layer[H][0]*f_layer[K][0]*f_layer[A][0];		r_layer[1]  = f_layer[H][0]*f_layer[K][1]*f_layer[A][0];
	r_layer[2]  = f_layer[H][0]*f_layer[K][2]*f_layer[A][0];		r_layer[3]  = f_layer[H][0]*f_layer[K][3]*f_layer[A][0];
	r_layer[4]  = f_layer[H][1]*f_layer[K][0]*f_layer[A][0];		r_layer[5]  = f_layer[H][1]*f_layer[K][1]*f_layer[A][0];
	r_layer[6]  = f_layer[H][1]*f_layer[K][2]*f_layer[A][0];		r_layer[7]  = f_layer[H][1]*f_layer[K][3]*f_layer[A][0];
	r_layer[8]  = f_layer[H][2]*f_layer[K][0]*f_layer[A][0];		r_layer[9]  = f_layer[H][2]*f_layer[K][1]*f_layer[A][0];
	r_layer[10] = f_layer[H][2]*f_layer[K][2]*f_layer[A][0];		r_layer[11] = f_layer[H][2]*f_layer[K][3]*f_layer[A][0];
	r_layer[12] = f_layer[H][3]*f_layer[K][0]*f_layer[A][0];		r_layer[13] = f_layer[H][3]*f_layer[K][1]*f_layer[A][0];
	r_layer[14] = f_layer[H][3]*f_layer[K][2]*f_layer[A][0];		r_layer[15] = f_layer[H][3]*f_layer[K][3]*f_layer[A][0];

	r_layer[16]  = f_layer[H][0]*f_layer[K][0]*f_layer[A][1];		r_layer[17]  = f_layer[H][0]*f_layer[K][1]*f_layer[A][1];
	r_layer[18]  = f_layer[H][0]*f_layer[K][2]*f_layer[A][1];		r_layer[19]  = f_layer[H][0]*f_layer[K][3]*f_layer[A][1];
	r_layer[20]  = f_layer[H][1]*f_layer[K][0]*f_layer[A][1];		r_layer[21]  = f_layer[H][1]*f_layer[K][1]*f_layer[A][1];
	r_layer[22]  = f_layer[H][1]*f_layer[K][2]*f_layer[A][1];		r_layer[23]  = f_layer[H][1]*f_layer[K][3]*f_layer[A][1];
	r_layer[24]  = f_layer[H][2]*f_layer[K][0]*f_layer[A][1];		r_layer[25]  = f_layer[H][2]*f_layer[K][1]*f_layer[A][1];
	r_layer[26]  = f_layer[H][2]*f_layer[K][2]*f_layer[A][1];		r_layer[27]  = f_layer[H][2]*f_layer[K][3]*f_layer[A][1];
	r_layer[28]  = f_layer[H][3]*f_layer[K][0]*f_layer[A][1];		r_layer[29]  = f_layer[H][3]*f_layer[K][1]*f_layer[A][1];
	r_layer[30]  = f_layer[H][3]*f_layer[K][2]*f_layer[A][1];		r_layer[31]  = f_layer[H][3]*f_layer[K][3]*f_layer[A][1];

	// calculate elements of defuzzifier layer & weight matrix//
	d_layer_sum = 0.0;
	for( i=0 ; i<RULE ; i++ ) d_layer_sum += r_layer[i];
	/// カウントするなどどの程度ここのルーチンに入るか要チェック ///
	if( d_layer_sum>-EPSILON_FUZZY && d_layer_sum<EPSILON_FUZZY ){
		if( d_layer_sum>=0.0 ) d_layer_sum =  EPSILON_FUZZY;
		else                   d_layer_sum = -EPSILON_FUZZY;
	}

	for( i=0 ; i<EMG_CH ; i++ ){
		d_layer[H][i] = 0.0;		d_layer[K][i] = 0.0;
		d_layer[A][i] = 0.0;
		
		for( j=0 ; j<RULE ; j++ ){
			d_layer[H][i] += ( weight->r_d[H][i][j]*r_layer[j] );
			d_layer[K][i] += ( weight->r_d[K][i][j]*r_layer[j] );
			d_layer[A][i] += ( weight->r_d[A][i][j]*r_layer[j] );
		}


		c_weight[H][i] = d_layer[H][i]/d_layer_sum;
		c_weight[K][i] = d_layer[K][i]/d_layer_sum;
		c_weight[A][i] = d_layer[A][i]/d_layer_sum;
	
		weight_p[H][i] = c_weight[H][i]*weight->ori[H][i];
		weight_p[K][i] = c_weight[K][i]*weight->ori[K][i];
		weight_p[A][i] = c_weight[A][i]*weight->ori[A][i];

	}

	if(learnFlag == ON){
	//力センサの値を元に脱ファジィ化層のウエイト値を学習
	LearningWeight(d_layer_sum, r_layer, rms, error_force, weight);
	}

	return 0;
}


/********************************************************
関数名：LearningWeight
説明　：力センサの値を元に脱ファジィ化層のウエイト値を学習
引数　：double d_layer_sum		　ルール層出力値の合計
		double r_layer[RULE]	  ルール層の出力値
		double rms[EMG_CH]		  筋電のrms値
　　　　double error_force[3]　　 人とロボットとの動きの差
		_weight *weight			　学習させるウエイト値
		
出力　：_weight *weight
********************************************************/
void LearningWeight(double d_layer_sum, double r_layer[RULE], double rms[EMG_CH], double error_force[3], _weight *weight){

	double temp[3];
	
	for(int i=0 ; i<EMG_CH ; i++ ){

		temp[H] = L_RATE_HIP*weight->ori[H][i]*rms[i]  *error_force[H]/d_layer_sum;
		temp[K] = L_RATE_KNE*weight->ori[K][i]*rms[i]  *error_force[K]/d_layer_sum;
		temp[A] = L_RATE_ANK*weight->ori[A][i]*rms[i]  *error_force[A]/d_layer_sum;
	
		for(int j=0 ; j<RULE ; j++ ){
			weight->r_d[H][i][j] += ( temp[H]*r_layer[j] );
			weight->r_d[K][i][j] += ( temp[K]*r_layer[j] );
			weight->r_d[A][i][j] += ( temp[A]*r_layer[j] );	
		}
	}
}


/********************************************************
関数名：Error_Force
説明　：人とロボットとの動きの差の計測
引数　：double forceData[3][2]		　力センサの値
		double error_force[3]		　人とロボットとの動きの差
		
出力　：double error_force[3]
********************************************************/
void Error_Force( double forceData[3][3], double error_force[3] ){
	
	//力センサの値が0.001以下の場合学習を終える
	for(int i=0;i<3;i++){
		if(forceData[i][X]<0.1 && forceData[i][X]>-0.1)	forceData[i][X] = 0;
	}

	//力センサの設置方向を変更したので　XにYの出力が入っている(データシート上は)
	error_force[H] = forceData[H][X]/LEN_H_HF;
	error_force[K] = forceData[K][X]/LEN_K_KF;
	error_force[A] = forceData[A][X]/LEN_A_AF;

}


/********************************************************
関数名：Read_Weight_Parameters
説明　：ファジィ制御に必要なウエイト値の読み込み
引数　：_weight *weight		　ファジィ制御に必要なウエイト値
		
出力　：_weight *weight
********************************************************/
void Read_Weight_Parameters( _weight *weight ){
	
	int i,j;
	FILE *fp;

	FILE_READ_OPEN( fp, "weight_data/fuzzifier.dat" );
	for( i=0 ; i<4 ; i++ ){
		for( j=0 ; j<3 ; j++ ) fscanf( fp, "%lf %lf ", &weight->dev[j][i], &weight->cen[j][i] );
	}
	fclose( fp );

	FILE_READ_OPEN( fp, "weight_data/defuzzifier_p.dat" );
	for( i=0 ; i<EMG_CH ; i++ ){
		for( j=0 ; j<RULE ; j++ )  fscanf( fp, "%lf %lf %lf\n", &weight->r_d[0][i][j], &weight->r_d[1][i][j], &weight->r_d[2][i][j] );
		
	}
	fclose( fp );

	FILE_READ_OPEN( fp, "weight_data/original_p.dat" );
	for( i=0 ; i<3 ; i++ ){
		for( j=0 ; j<EMG_CH ; j++ )		fscanf( fp, "%lf ", &weight->ori[i][j] );
	}
	fclose( fp );

	
}


/********************************************************
関数名：Read_Weight_Parameters
説明　：ファジィ制御に必要なウエイト値の書き込み
引数　：_weight *weight		　ファジィ制御に必要なウエイト値
		
出力　：_weight *weight
********************************************************/
void Write_Weight_Parameters( _weight *weight ){

	int i,j;
	FILE *fp;

	FILE_WRITE_OPEN( fp, "weight_data/fuzzifier.dat" );
	for( i=0 ; i<4 ; i++ ){
		for( j=0 ; j<3 ; j++ ) fprintf( fp, "%+12.6lf %+12.6lf\n ", weight->dev[j][i], weight->cen[j][i] );
	}
	fclose( fp );

	FILE_WRITE_OPEN( fp, "weight_data/defuzzifier_p.dat" );
	for( i=0 ; i<EMG_CH ; i++ ){
		for( j=0 ; j<RULE ; j++ ) fprintf( fp, "%+12.6lf %+12.6lf %+12.6lf\n", weight->r_d[0][i][j], weight->r_d[1][i][j] ,weight->r_d[2][i][j] );
	}
	fclose( fp );

	
	FILE_WRITE_OPEN( fp, "weight_data/original_p.dat" );
	for( i=0 ; i<3 ; i++ ){
		for( j=0 ; j<EMG_CH ; j++ ) fprintf( fp, "%+12.6lf\n ", weight->ori[i][j] );
	}
	fclose( fp );

	return;
}


/********************************************************
関数名：Sig_Func
説明　：メンバシップ関数用の関数の計算　（シグモイド関数）
引数　：double theta		　関節角度
		double w_o			　関数を横軸方向に平行移動させるための変数
		double w_i			　関数の傾きを変更させる変数
		
戻り値：( 1.0/( 1.0+exp( -theta*w_i-w_o ) ) );
********************************************************/
inline double Sig_Func( double theta, double w_o, double w_i ){

	return ( 1.0/( 1.0+exp( -theta*w_i-w_o ) ) );
}


/********************************************************
関数名：Gau_Func
説明　：メンバシップ関数用の関数の計算　（ガウシアン関数）
引数　：double theta		　関節角度
		double w_o			　関数を横軸方向に平行移動させるための変数
		double w_i			　関数の傾きを変更させる変数
		
戻り値：( exp( -(w_o+theta)*(w_o+theta)/(w_i*w_i) ) );
********************************************************/
inline double Gau_Func( double theta, double w_o, double w_i ){

	return ( exp( -(w_o+theta)*(w_o+theta)/(w_i*w_i) ) );
}