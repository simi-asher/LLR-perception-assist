/*****************************************************************************************************************************
******************************************************************************************************************************
レーザーレンジファインダ(URG-04LX)使用に必要な関数をまとめたソースファイル
ソフトウェア：urg_library-1.2.0使用


******************************************************************************************************************************
*****************************************************************************************************************************/

#include <algorithm> 
#include "urg.h"
//simi
/*#include <iostream>
#include <fstream>
using namespace std;
ofstream fil;
fil.open("bump.dat",ios::out | ios::binary);*/

/********************************************************
関数名：Calculate_URG
説明　：メインループの中で呼び出す関数
		URGのデータ処理や物体検出はこの関数内で行う
引数　：urg_t *urg							URGの設定用の構造体
	　　long *length_data					距離データ
	    const int *length_data_size			データの個数(for文や領域確保に使用)
		int *time							計測にかかった時間
		struct URG_data *urgData			距離データをX-Y座標に変換した値(平滑化などの処理も含む)
		struct Obstacle_data *obstacleData	物体検出後のデータ(障害物の高さなど)

出力　：struct Obstacle_data *obstacleData
********************************************************/
void Calculate_URG(urg_t *urg, long *length_data, int *length_data_size, int *time, struct URG_data *urgData, struct Obstacle_data *obstacleData){
	
	
	//距離の計測	 Distance measurement
	//マルチスレッド内で使用しているためこの関数内ではコメントアウトしている
	//Measure_URG(urg, length_data, length_data_size, time);

	//計測データの処理
	ChangeZeroPoint(length_data, *length_data_size); //Changes distance to every pt that is less than 4cm and greater than 4m is 0
	
	ChangeCoordinate(urg, length_data, *length_data_size, urgData); //Convert distance data to X-Y coordinates. front is positive. i goes from front to back

	DeleateOfNotMeasureData(*length_data_size, urgData); //if X&Y is 0 then make same as neighbor
	
	FromSensorToGround(URG_ON, urg, *length_data_size, urgData); //obstacle y is -ve. Convert the origin from the sensor position to the ground directly below the sensor

	//生データでもある程度の精度で検出できたのでフィルターは使用していない
	Smoothing_Data(NonFilter, *length_data_size, urgData); //nothing for non-filter

	//探索用にデータの修正
	ChangeData(urgData, urg, *length_data_size, obstacleData); //simi array ordering centre->front->center->back,y sign reversed to make positive 

	//障害物の探索
	DetectLine_SAM(obstacleData, *length_data_size);

	printf(" %lf\n",obstacleData[0].distance); //simi uncommented
}


/********************************************************
関数名：Init_URG
説明　：メインループに入る前に呼び出す関数(メモリ領域を確保する前に呼び出すこと)
		URGの初期設定
引数　：urg_t *urg						URGの設定用の構造体
	　　const long connect_baudrate		通信速度
	    const char connect_device[]		ポート番号

出力　：urg_t *urg
********************************************************/
void Init_URG(urg_t *urg, const long connect_baudrate, const char connect_device[]){
	int ret=0;
	int first_step;
	int last_step;

// URGの接続と各種設定
	ret = urg_open(urg, URG_SERIAL, connect_device, connect_baudrate);
	if(ret<0){
		printf("URGが接続されていません\nコンポートを確認してください\n\n");
		getchar();
		exit(-1);
	}
	first_step = urg_deg2step(urg, URG_FirstDeg);
	last_step  = urg_deg2step(urg, URG_LastDeg);
	urg_set_scanning_parameter(urg, first_step, last_step, SkipStep);
}


/********************************************************
関数名：Measure_URG
説明　：距離の計測
引数　：urg_t *urg						URGの設定用の構造体
	　　long *length_data				距離データ
	    int *length_data_size			データの個数
		int *time						計測にかかった時間

出力　：long *length_data
		int *length_data_size
		int *time
********************************************************/
void Measure_URG(urg_t *urg, long *length_data, int *length_data_size, int *time){
	long timestamp;
	static long preTimestamp=0;

	//距離を計測する
	urg_start_measurement(urg, URG_DISTANCE, ScanTimes, SkipScan);
	//データの長さ
    *length_data_size = urg_get_distance(urg, length_data, &timestamp);

	//計測にかかった時間
	*time = timestamp - preTimestamp;
	
	preTimestamp = timestamp;
}


/********************************************************
関数名：KeepData
説明　：データの確保
引数　：struct URG_data *beforeData		残しておきたいデータ
	    const int length_data_size		データの個数
		struct URG_data *afterData		確保用のデータ

出力　：struct URG_data *afterData
********************************************************/
void KeepData(struct URG_data *beforeData, const int length_data_size, struct URG_data *afterData){
	
	for(int i=0; i<length_data_size; i++){
		afterData[i].x = beforeData[i].x;	afterData[i].y = beforeData[i].y;
	}
}



/********************************************************
関数名：EndMeasure
説明　：メインループ終了後に呼び出す関数(メモリ領域を開放する前に呼び出すこと)
		URGの接続を閉じる
引数　：urg_t *urg						URGの設定用の構造体　
********************************************************/
void EndMeasure(urg_t *urg){

	urg_close(urg);		//URGの接続を閉じる
}


/********************************************************
関数名：ChangeZeroPoint
説明　：計測範囲外を0へ（40mm～4m）
引数　：long *length_data				距離データ
	    const int *length_data_size		データの個数

出力　：long *length_data
********************************************************/
void ChangeZeroPoint(long *length_data, const int length_data_size){
	for(int i=0; i<length_data_size; ++i){
		if(length_data[i]<40 || length_data[i]>4000)
			length_data[i] = 0;
	}
}


/********************************************************
関数名：ChangeCoordinate
説明　：距離データをX-Y座標に変換
		またセンサ正面がlength_data_size=0になるように変換
		Convert distance data to X-Y coordinates
		Also convert the front of the sensor to length_data_size = 0
引数　：urg_t *urg						URGの設定用の構造体
		long *length_data				距離データ
	    const int *length_data_size		データの個数
		struct URG_data *urgData		距離データをX-Y座標に変換した値(※重要)

出力　：struct URG_data *urgData
********************************************************/
void ChangeCoordinate(urg_t *urg, long *length_data, const int length_data_size, struct URG_data *urgData){
	double radian;

	for(int i=0; i<length_data_size; ++i){
		
		radian = urg_index2rad(urg, i*SkipStep);
		
		urgData[i].rawX = -(length_data[i] * sin(radian))/1000; //-(length_data[i]*cos(radian))/1000
		urgData[i].rawY =  (length_data[i] * cos(radian))/1000; //length_data[i]*sin(radian))/1000
		//printf("%d %lf %lf\n", i, urgData[i].rawX, urgData[i].rawY); 
	}
	//printf("\n");
	
}


/********************************************************
関数名：DeleateOfNotMeasureData
説明　：計測範囲外のデータ(x=y=0)を削除する
引数　：const int *length_data_size		データの個数
		struct URG_data *urgData		距離データをX-Y座標に変換した値(※重要)

出力　：struct URG_data *urgData
********************************************************/
void DeleateOfNotMeasureData(const int length_data_size, struct URG_data *urgData){
	for(int i=length_data_size-2; i>=0; i--){
		if(urgData[i].rawX==0 && urgData[i].rawY==0){
			urgData[i].rawX = urgData[i+1].rawX;
			urgData[i].rawY = urgData[i+1].rawY;
		}
	}
}


/********************************************************
関数名：FromSensorToGround
説明　：原点をセンサ位置からセンサ真下の地面に変換(ON:地面原点, OFF:センサ原点)
引数　：const int flag					ON or OFF(座標変換をするかどうか)
		urg_t *urg						URGの設定用の構造体
		const int *length_data_size		データの個数
		struct URG_data *urgData		距離データをX-Y座標に変換した値(※重要)

出力　：struct URG_data *urgData
********************************************************/
void FromSensorToGround(const int flag, urg_t *urg, const int length_data_size, struct URG_data *urgData) {
	double translation[2];

	//センサ正面（真下）のインデックス番号を取得
	int front_index = urg_deg2step(urg, (URG_FrontDeg - URG_FirstDeg) / SkipStep);
	if (front_index*2 == length_data_size) //simi *2
	{
		front_index = front_index - 1;
		//printf("yes");
	}
	//printf("%d %d\n", length_data_size, front_index);
	//センサ真下X-Y座標
	translation[X] = urgData[front_index].rawX;	translation[Y] = urgData[front_index].rawY;

	switch(flag){
	//座標変換あり
	case ON:
	for(int i=length_data_size-1; i>=0; i--){
		urgData[i].x = urgData[i].rawX - translation[X];
		urgData[i].y = urgData[i].rawY - translation[Y];
		//printf("raw %d %lf %lf\n", i, urgData[i].x, urgData[i].y); //print
	}
	break;

	//座標変換なし
	case OFF:
	for(int i=length_data_size-1; i>=0; i--){
		urgData[i].x = urgData[i].rawX;
		urgData[i].x = urgData[i].rawY;
	}
	break;

	default:
	break;
	}

}


/********************************************************
関数名：Smoothing_Data
説明　：原点をセンサ位置からセンサ真下の地面に変換(ON:地面原点, OFF:センサ原点)
引数　：const int filterMode			フィルタの設定(基本NonFilterかDelayFilter)
		const int *length_data_size		データの個数
		struct URG_data *urgData		距離データをX-Y座標に変換した値(※重要)

出力　：struct URG_data *urgData
********************************************************/
void Smoothing_Data(const int filterMode, const int length_data_size, struct URG_data *urgData){
	double sum[2]={0};
	static double pre1[2][1000]={0}, pre2[2][1000]={0};

	switch(filterMode){

	/*************************************************************************
	//					NonFilter
	//					フィルタなし
	**************************************************************************/
	case NonFilter:
	for(int i=0; i<length_data_size; i++){
		urgData[i].x = urgData[i].x;
		urgData[i].y = urgData[i].y;
	}

	break;

	/*************************************************************************
	//					AveragingFilter
	//					平均化フィルタ
	**************************************************************************/
	case AveragingFilter:

	urgData[0].x = (urgData[0].x + urgData[1].x)/2;
	urgData[0].y = (urgData[0].y + urgData[1].y)/2;
	urgData[1].x = (urgData[0].x + urgData[1].x + urgData[2].x)/3;
	urgData[1].y = (urgData[0].y + urgData[1].y + urgData[2].y)/3;
		
	for(int j=2;j<length_data_size-2;j++){
		for(int k=-2;k<=2;k++){
			sum[X] += urgData[j+k].x;
			sum[Y] += urgData[j+k].y;
		}
		urgData[j].x = sum[X]/5;
		urgData[j].y = sum[Y]/5;
		sum[X] = 0;
		sum[Y] = 0;
	}
	
	urgData[length_data_size-2].x = (urgData[length_data_size-3].x + urgData[length_data_size-2].x + urgData[length_data_size-1].x)/3;
	urgData[length_data_size-2].y = (urgData[length_data_size-3].y + urgData[length_data_size-2].y + urgData[length_data_size-1].y)/3;
	urgData[length_data_size-1].x = urgData[length_data_size-1].x;
	urgData[length_data_size-1].y = urgData[length_data_size-1].y;
	break;


	/*************************************************************************
	//					GaussianFilter
	//					ガウシアンフィルタ
	**************************************************************************/
	case GaussianFilter:

		urgData[0].x = (2*urgData[0].x + urgData[1].x)/3;
		urgData[0].y = (2*urgData[0].y + urgData[1].y)/3;
	

	for(int j=1;j<length_data_size-1;j++){
		sum[X] = 1*urgData[j-1].x + 7*urgData[j].x + 1*urgData[j+1].x;
		sum[Y] = 1*urgData[j-1].y + 7*urgData[j].y + 1*urgData[j+1].y;
		urgData[j].x = sum[X]/9;
		urgData[j].y = sum[Y]/9;
		sum[X] = 0;		sum[Y] = 0;
	}
	
	urgData[length_data_size-1].x = urgData[length_data_size-1].x;
	urgData[length_data_size-1].y = urgData[length_data_size-1].y;

	break;

	/*************************************************************************
	//					DelayFilter
	**************************************************************************/
	case DelayFilter:
		for(int i=0; i<length_data_size; i++){
			sum[X] = urgData[i].x + pre1[X][i] + pre2[X][i];
			sum[Y] = urgData[i].y + pre1[Y][i] + pre2[Y][i];
			urgData[i].x = sum[X]/3;	urgData[i].y = sum[Y]/3;
			
			pre2[X][i] = pre1[X][i];	pre2[Y][i] = pre1[Y][i]; 
			pre1[X][i] = urgData[i].x;	pre1[Y][i] = urgData[i].y;		
		}
		
	
	break;

	/*************************************************************************
	//					default処理
	//					強制終了(-1)
	**************************************************************************/
	default:
	printf("ERROR Smoothing_Obstacle : Filter mode を選択してください");
	exit(-1);
	}
}


/********************************************************
関数名：ChangeData
説明　：配列の先頭に原点データが来るように変更
引数　：struct URG_data *urgData
	　　const int length_data_size
	    struct Obstacle_data *obstacleData

出力　：struct Obstacle_data *obstacleData
********************************************************/
void ChangeData(struct URG_data *urgData, urg_t *urg, const int length_data_size, struct Obstacle_data *obstacleData) {
	//障害物がURGより低い位置にある場合URGの距離データは地面より小さくなるのでY座標の値には-をつけている
	//URGより高い場合を現在考慮していないため必要に応じて修正が必要
	// If the obstacle is at a position below the URG, the URG distance data is smaller than the ground, so the Y coordinate value is marked with-
	// If higher than URG is not considered at present, correction is necessary if necessary
	//simi no reordering index
	for (int i = 0; i < length_data_size; i++) {
		obstacleData[i].x = urgData[i].x;
		obstacleData[i].y = -urgData[i].y;
	}
	////simi with reordering
	//int front_index = urg_deg2step(urg, (URG_FrontDeg - URG_FirstDeg) / SkipStep); //simi
	////for(int i=length_data_size-1;i>=0;i--){
	//for (int i = front_index; i >=0; i--)
	//{
	//	//obstacleData[length_data_size-1-i].x = urgData[i].x;	
	//	//obstacleData[length_data_size-1-i].y = -urgData[i].y;
	//	//simi no change in order, only rev sign.
	//	obstacleData[-i+front_index].x = urgData[i].x;
	//	obstacleData[-i+front_index].y= -urgData[i].y;
	//}
	////simi old way of going back->center like rotation
	//for (int i = front_index+1; i < length_data_size; i++) {
	////	obstacleData[i].x = urgData[length_data_size-i+front_index+1].x;
	////	obstacleData[i].y = -urgData[length_data_size - i + front_index + 1].y;
	//	obstacleData[i].x = urgData[i].x;
	//	obstacleData[i].y = -urgData[i].y;
	//}
	
	//for (int i = 0; i<length_data_size; i++) {
	//	printf("mod %d %lf %lf \n", i, obstacleData[i].x, obstacleData[i].y); //print
	//}
}


/********************************************************
関数名：DetectLine_SAM
説明　：Split and Merge法を使用
引数　：struct Obstacle_data *obstacleData
	　　int data_size
出力　：


********************************************************/
//以下DetectLine_SAMに必要な関数群
void DetectLine_SAM(struct Obstacle_data *obstacleData, int data_size){
	int FINISH_POINT = 0;
	int START_POINT = 0;
	double max=0;
	int max_num=0;
	int point_num=2;
	int prePoint_num;
	int data_num=2*point_num-1;
	int i = 0,j=0;
	//適当に配列20確保している　エラーが出るなら対処が必要かも Appropriate array 20 has been secured
	struct Line_data line[100] = { 0 }, new_line[100] = { 0 };

	//current change
	//DetectObject(obstacleData, data_size, &FINISH_POINT); // End the search if the distance between adjacent points is greater than or equal to OBS_THRESHOLE for the second time i.e. end of step
	//simi find start and end point before SAM
	for (i = data_size - 1; i >= 1; i--)
	{
		if (abs(obstacleData[i].x) < 1.3 ) //check &&  obstacleData[i].y < 0.1
		{
			FINISH_POINT = i;
			break;
		}

	}
	for (i = 1; i <data_size - 1; i++)
	{
		if (abs(obstacleData[i].y- obstacleData[i-1].y)>0.1)
		{
			START_POINT = i-1;
			break;
		}
		//printf("slope  %d %lf %lf %lf\n", i, obstacleData[i].x, obstacleData[i].y,(obstacleData[i + 1].y - obstacleData[i].y) / (obstacleData[i + 1].x - obstacleData[i].x)); //printcur
	}
	//printf("%d %lf %lf %d %lf %lf\n", START_POINT, obstacleData[START_POINT].x, obstacleData[START_POINT].y, FINISH_POINT, obstacleData[FINISH_POINT].x, obstacleData[FINISH_POINT].y);
	InitDetectCornerPoint(obstacleData, line, &point_num, START_POINT, FINISH_POINT); //added start point // Connect the start point and end point with a straight line and calculate the distance between the straight line and each point
	// If the maximum distance is greater than or equal to the threshold, that point is taken as a corner
	//point_num=3 if threshold crossed else 2
	//探索のためのループスタート  Loop start for search
	for(int n=0;n<5;n++){//theoretical max of data_num is 129 at n=5
	prePoint_num = point_num;
	data_num = 2*point_num-1;
	
	
	UpdataValue(line, new_line, data_num); //create new array and fill in 0 and even indexes


	DetectCornerPoint(obstacleData, line, new_line, &point_num, data_num); //if the maximum difference between the line segment obtained in the previous search and the points in the interval is greater than or equal to the threshold, that point is taken as a corner
	
	
	//値の更新 Update value
	for(j=0; j<data_num; j++){
		line[j].data_num = new_line[j].data_num;
		line[j].x = new_line[j].x;
		line[j].y = new_line[j].y;

		if(j<data_num-1){
		line[j].grad = new_line[j].grad;
		line[j].intr = new_line[j].intr;
		
		}
		
	}
	//printf("%d %d %d %d\n", n, point_num,prePoint_num, data_num); //simi
	if(point_num == prePoint_num) break;

	}
	//探索終了 End search
	
	FittingLine(obstacleData, line, point_num); //pickup //Line fitting by least squares method
	//printf("%d \n", data_num); //simi
	for(i=0;i<data_num;i++){
	obstacleData[i].x = line[i].x;
	obstacleData[i].y = line[i].y;
	obstacleData[i].size = point_num;
	//printf("line[%d]: %lf  %lf\n",line[i].data_num, line[i].x,line[i].y); //print
	}
	for(i=0;i<data_num-1;i++){
		if(line[i+1].data_num != line[i].data_num){
			//printf("line[%d]: %lf  %lf\n",line[i].data_num, line[i].grad,line[i].intr);
			//printf("%lf,  %lf\n", line[i].x, line[i].y); //print
		}
	}
	//printf("%d \n", point_num); //current
	BumpRecognition(obstacleData, point_num);
	
}

//隣接する点の距離が閾値以上の場合探索を終了する（OBS_THRESHOLE=0.1m）
void DetectObject(struct Obstacle_data *obstacleData, int data_size, int *FINISH_POINT){
	double distance=0,a,b;
	int flag = 0;

	for(int i=0;i<data_size;i++){
	a = (obstacleData[i + 1].x - obstacleData[i].x)*(obstacleData[i + 1].x - obstacleData[i].x);
	b = (obstacleData[i + 1].y - obstacleData[i].y)*(obstacleData[i+1].y-obstacleData[i].y);
	distance = sqrt(a + b);
	//if(i>=10 && i<=16){
	//printf("%d %lf %lf %lf \n", i, distance,obstacleData[i].x, obstacleData[i].y);//simi //print
	//}
	if(distance > OBS_THRESHOLE){
		if (flag == 0) {
			//printf("%d %d \n",i,data_size);
			flag = 1;//simi
			continue;
		}
		else {
			//printf("end %d %d \n", i, data_size);
			*FINISH_POINT = i;
			break;
		}
	/*for (int i = 25; i<data_size; i++) {
		a = obstacleData[i + 1].y;
		b = obstacleData[i].y;
		if (a != 0 && abs(a)>(1 + OBS_THRESHOLE)*abs(b)) {
			//printf("%d %d %d \n", i, abs(a),abs(b)); //simi
			*FINISH_POINT = i - 1; //simi i
			break;
		*/
		}
	}
}

//始点と終点を直線で結び，その直線と各点との距離を計算
//最大距離が閾値以上のときその点をコーナーとする
// Connect the start point and end point with a straight line and calculate the distance between the straight line and each point
// If the maximum distance is greater than or equal to the threshold, that point is taken as a corner
void InitDetectCornerPoint(struct Obstacle_data *obstacleData, struct Line_data *line, int *point_num, const int START_POINT, const int FINISH_POINT){

	double error,max=0;
	int max_num=0;
	int data_num=2*(*point_num)-1;
	
	//初期の直線
	line[0].data_num = START_POINT;	line[data_num-1].data_num = FINISH_POINT; //simi START_POINT prev 0

	line[0].x = obstacleData[START_POINT].x;	line[0].y = obstacleData[START_POINT].y; //simi START_POINT prev 0
	line[data_num-1].x = obstacleData[FINISH_POINT].x;	line[data_num-1].y = obstacleData[FINISH_POINT].y;

	line[0].grad = (line[data_num-1].y-line[0].y)/(line[data_num-1].x-line[0].x);
	line[0].intr = (line[data_num-1].x*line[0].y-line[0].x*line[data_num-1].y)/(line[data_num-1].x-line[0].x);

	//直線の探索
	for(int i = line[0].data_num; i < line[data_num-1].data_num; i++){
		error = abs(line[0].grad*obstacleData[i].x - obstacleData[i].y + line[0].intr)/sqrt(line[0].grad*line[0].grad+1);
		//printf("error[%d]:%lf\n",i,error);
		if(max<error){
			max_num = i;
			max = error;
		}
	}
	if(max>SPLIT_THRESHOLD){
		line[1].data_num = max_num;
		line[1].x = obstacleData[max_num].x;	line[1].y = obstacleData[max_num].y;
		*point_num+=1;
		
	}
	for(int i=0;i<data_num-1;i++){
		line[i].grad = (line[i+1].y-line[i].y)/(line[i+1].x-line[i].x);
		line[i].intr = (line[i+1].x*line[i].y-line[i].x*line[i+1].y)/(line[i+1].x-line[i].x);
		//printf("%d \n", data_num);
		//printf("%lf  %lf\n",line[i].grad,line[i].intr);
	}
}

//値の更新 // Update value
void UpdataValue(struct Line_data *line, struct Line_data *new_line, const int data_num){

	for(int i=0; i<data_num; i+=2){
		new_line[i].data_num = line[i/2].data_num;
		new_line[i].x        = line[i/2].x;
		new_line[i].y        = line[i/2].y;
		//odd indexes of new_line empty (1,3,...)
		if(i<data_num-1){
		new_line[i].grad = line[i/2].grad;
		new_line[i].intr = line[i/2].intr;
		//printf("%lf  %lf\n",new_line[i].grad,new_line[i].intr);
		}
	}
}

//以前の探索で求めた線分とその区間内の点との差の最大値の探索
//最大距離が閾値以上のときその点をコーナーとする
// Search for the maximum difference between the line segment obtained in the previous search and the points in the interval
// If the maximum distance is greater than or equal to the threshold, that point is taken as a corner
void DetectCornerPoint(struct Obstacle_data *obstacleData, struct Line_data *line, struct Line_data *new_line, int *point_num, const int data_num){
	
	double error,max=0;
	int max_num=0;


	//直線の探索 // Line search
	for(int i=0; i<data_num; i+=2){
		max = 0;
		

		//以前の探索で求めた線分とその区間内の点との差の最大値の探索 Search for the maximum difference between a line segment obtained in a previous search and a point in that interval
		
		for(int j = new_line[i].data_num; j < new_line[i+2].data_num; j++){
			if(i==data_num-1)	break;
			error = abs(new_line[i].grad*obstacleData[j].x - obstacleData[j].y + new_line[i].intr)/sqrt(new_line[i].grad*new_line[i].grad+1);
			//printf("error[%d]:%lf\n",j,error); 
			if(max<error){
				max_num = j;
				max = error;
			}
		}
		
	
	//探索した最大値が閾値を越えているか(越えていたらその点をコーナーとする) Whether the searched maximum value exceeds the threshold (if it exceeds, the point is taken as a corner)
	if(max>SPLIT_THRESHOLD){
		new_line[i+1].data_num = max_num;
		new_line[i+1].x = obstacleData[max_num].x;	new_line[i+1].y = obstacleData[max_num].y;
		*point_num+=1;
	//越えていないときは一つ前の点の値を代入(後で削除する) // If not, substitute the value of the previous point (delete later)
	}else{
		new_line[i+1].data_num = new_line[i].data_num;
		new_line[i+1].x = new_line[i].x;	new_line[i+1].y = new_line[i].y;
	}
	}
	//コーナーであると検出された点を用いて直線を求める Find a straight line using points detected to be corners
	for(int i=0;i<data_num-1;i++){
		if(new_line[i+1].data_num != new_line[i].data_num){
			new_line[i].grad = (new_line[i+1].y-new_line[i].y)/(new_line[i+1].x-new_line[i].x);
			new_line[i].intr = (new_line[i+1].x*new_line[i].y-new_line[i].x*new_line[i+1].y)/(new_line[i+1].x-new_line[i].x);			
		}else{
			new_line[i].grad = (new_line[i+2].y-new_line[i].y)/(new_line[i+2].x-new_line[i].x);
			new_line[i].intr = (new_line[i+2].x*new_line[i].y-new_line[i].x*new_line[i+2].y)/(new_line[i+2].x-new_line[i].x);	
		}
		
	}

	new_line[0].flag = true;
	for(int i=1;i<data_num;i++){
		if(new_line[i].data_num != new_line[i-1].data_num){
			new_line[i].flag = true;
		}else new_line[i].flag = false;
	}
	

	//必要のない配列の削除 // Delete unnecessary arrays
	for(int i=1; i<data_num; i++){
		if(new_line[i].flag == false){
			for(int j=i; j<data_num; j++){
				new_line[j].flag = new_line[j+1].flag;
				new_line[j].data_num = new_line[j+1].data_num;
				new_line[j].x = new_line[j+1].x;
				new_line[j].y = new_line[j+1].y;
				new_line[j].grad = new_line[j+1].grad;
				new_line[j].intr = new_line[j+1].intr;
			}
		}
	}
	
}

//最小二乗法による直線のフィッティング // Line fitting by least squares method
void FittingLine(struct Obstacle_data *obstacleData, struct Line_data *line, int point_num) {
	int num_ave;

	//合計，平均，分散，共分散，傾き，切片 Sum, mean, variance, covariance, slope, intercept
	double sumX, sumY, aveX, aveY, var, cov, a, b;

	for (int i = 0; i < point_num - 1; i++) {
		num_ave = line[i + 1].data_num - line[i].data_num + 1;

		sumX = sumY = var = cov = 0;


		for (int j = line[i].data_num; j < line[i + 1].data_num + 1; j++) {
			sumX += obstacleData[j].x;	sumY += obstacleData[j].y;
		}
		aveX = sumX / num_ave;	aveY = sumY / num_ave;
		for (int j = line[i].data_num; j < line[i + 1].data_num + 1; j++) {
			var += (obstacleData[j].x - aveX)*(obstacleData[j].x - aveX);
			cov += (obstacleData[j].x - aveX)*(obstacleData[j].y - aveY);
		}
		var /= (num_ave - 1);	cov /= num_ave;

		a = cov / var;	b = aveY - a * aveX;




		obstacleData[i].grad = a;
		obstacleData[i].intr = b;
		//printf("obs: %lf  %lf\n",obstacleData[i].grad,obstacleData[i].intr);
	}
	CalculateIntersection(obstacleData, line, point_num);
	//for (int i = 0; i < point_num; i++) {
	//	printf("obs:%d %lf  %lf\n", i, obstacleData[i].bumpX, obstacleData[i].bumpY); //print
	//}
}

//交点の計算 Intersection calculation
void CalculateIntersection(struct Obstacle_data *obstacleData, struct Line_data *line, int point_num) {

	//始点
	obstacleData[0].bumpX = line[0].x;	obstacleData[0].bumpY = obstacleData[0].grad*line[0].x + obstacleData[0].intr;
	//交点
	for (int i = 1; i < point_num - 1; i++) {
		obstacleData[i].bumpX = -(obstacleData[i - 1].intr - obstacleData[i].intr) / (obstacleData[i - 1].grad - obstacleData[i].grad);
		obstacleData[i].bumpY = (obstacleData[i - 1].grad*obstacleData[i].intr - obstacleData[i].grad*obstacleData[i - 1].intr) / (obstacleData[i - 1].grad - obstacleData[i].grad);
	}
	//終点
	obstacleData[point_num - 1].bumpX = line[point_num - 1].x;	obstacleData[point_num - 1].bumpY = obstacleData[point_num - 2].grad*line[point_num - 1].x + obstacleData[point_num - 2].intr;
	for (int i = 0; i < point_num; i++) { //print
		//printf("%lf, %lf\n", obstacleData[i].bumpX, obstacleData[i].bumpY); //print
	}
}

//特徴点の検出（障害物の高さ，奥行き，障害物までの距離の計算用） Feature point detection (for calculating obstacle height, depth, distance to obstacle)
void BumpRecognition(struct Obstacle_data *obstacleData, int point_num) {
	int flag = 0;
	int temp = 0;
	//double pos[25][2];
	//int max = 0;
	//int max_num = 0;
	int start = 0, fin = 0;
	////std::sort(obstacleData->bumpX, obstacleData->bumpX+point_num);
	//if (obstacleData[0].bumpX < -0.1) {
	//	flag = 2;
	//	start = 0;
	//	fin = 0;
	//}
	/*else if (obstacleData[0].bumpX < 0.75 && obstacleData[0].bumpY > 0.15) {
		start = 0;
		flag = 1;
	}*/
	for (int i = 0; i < point_num - 1; i++) {
		//pos[i][0] = obstacleData[i].bumpX;
		//pos[i][1] = obstacleData[i].bumpY;
		if (flag == 0 && i != 0 && obstacleData[i].bumpY < -0.1 && obstacleData[i].bumpX < 0 && obstacleData[i].bumpX > -0.5) {
			fin = i - 1;
			flag = 3; //meaning desc end pt found
		}
	}
	for (int i = 0; i < point_num - 1; i++) {
		//pos[i][0] = obstacleData[i].bumpX;
		//pos[i][1] = obstacleData[i].bumpY;
		//if (flag == 0 && i!=0 && obstacleData[i].bumpY < -0.1 && obstacleData[i].bumpX < 0) {
		//	fin = i-1;
		//	flag = 3; //meaning desc end pt found
		//}
		//else 
		if (flag == 0 && obstacleData[i].bumpY > 0.11 && obstacleData[i].bumpX < 0.65 && obstacleData[i].bumpX >0 ) {
			start = i;
			flag = 1; //meaning  asc start found
		}
		else if (flag == 0 && obstacleData[i].bumpX < -0.2 && abs(obstacleData[i].bumpY - obstacleData[i + 1].bumpY) >0.12) {
			start = i;
			flag = 4; //meaning sitting obs
		}
		else if (flag == 1 && obstacleData[i].bumpX > 0.1 && abs(obstacleData[i].bumpY - obstacleData[i + 1].bumpY) > 0.12)
		{
			fin = i;
			//printf("f %d \n", fin);
			flag = 2; //meaning asc end found
		}
		
		printf("%d %lf, %lf, %lf %d\n", i, obstacleData[i].bumpX, obstacleData[i].bumpY, abs(obstacleData[i].bumpY - obstacleData[i + 1].bumpY), flag); //printcur
	}
	//std::sort(pos, pos + point_num);
	//for (int i = 0; i < point_num; i++) {
	//	printf("%d %lf, %lf\n", i, pos[i][0], pos[i][1]);
	//}
	//printf("%d %d\n", start, fin);
	if (flag==1 && obstacleData[start].bumpX > 0 && obstacleData[fin].bumpX > 0 && obstacleData[start].bumpX < obstacleData[fin].bumpX)
	{
		temp = fin;
		fin = start;
		start = temp;
		flag = 2;
	}
	//if descending

	if (flag == 3) {
		
		for (int i = 0; i < fin; i++) {
			printf("%d", i);
			if (obstacleData[i].bumpX < 0.3 && abs(obstacleData[i].bumpY - obstacleData[i + 1].bumpY)>0.1) {
				start = i+1;
				
				flag = 5;
				break;
			}
				
		}
	} 
	if (flag == 4) {//for sitting
		for (int i = start+1; i < point_num - 1; i++) { //-1?
			if (obstacleData[i].bumpX >-0.7 && obstacleData[i].bumpY >0.3) {
				fin = i;
				flag = 6;
				//break;
			}

		}
	}
	printf("%d %d %d\n", start, fin,flag);
	//printcur
   //obstacleData[0].bump_p = max_num;
   //obstacleData[0].height = obstacleData[max_num].bumpY;

   //obstacleData[0].distance = obstacleData[max_num].bumpX;
	if (flag==5) { //desc obstacle behind (obstacleData[start].bumpX < 0
		obstacleData[0].bump_p = start;
		obstacleData[0].height = -obstacleData[fin].bumpY+ obstacleData[fin+1].bumpY;
		obstacleData[0].distance = obstacleData[start].bumpX;
		obstacleData[0].depth = obstacleData[fin].bumpX - obstacleData[start].bumpX;
		obstacleData[0].depth = -0.27;
		//obstacleData[0].depth = obstacleData[max_num + 1].bumpX - obstacleData[max_num].bumpX; //should be negative

	}
	else if (flag == 2) { //obstacle in front
		obstacleData[0].bump_p = start;
		obstacleData[0].height = max(obstacleData[start].bumpY, obstacleData[fin].bumpY) - obstacleData[fin + 1].bumpY; //obstacleData[fin].bumpY  - obstacleData[fin + 1].bumpY;; //fin-ground next to fin
		obstacleData[0].distance = obstacleData[fin].bumpX;
		if ((obstacleData[fin].bumpX - obstacleData[fin + 1].bumpX) < 0.05){ //the point after the end of the bump is near the bump
			obstacleData[0].depth = obstacleData[start].bumpX - min(obstacleData[fin + 1].bumpX, obstacleData[fin].bumpX);
		}
		else {//if not then just calc diff b/w start and fin X
			obstacleData[0].depth = obstacleData[start].bumpX - obstacleData[fin].bumpX;
		}
		//obstacleData[0].depth = obstacleData[max_num - 1].bumpX - obstacleData[max_num].bumpX;
	}
	else if (flag == 6) {
		obstacleData[0].bump_p = start;
		obstacleData[0].height = obstacleData[fin].bumpY;
		obstacleData[0].distance = obstacleData[start].bumpX;
		obstacleData[0].depth = obstacleData[fin].bumpX - obstacleData[start].bumpX; 
		//if (abs(obstacleData[0].depth) < 0.1)
		obstacleData[0].depth = -0.45;
	}
	printf("%lf %lf %lf\n", obstacleData[0].height, obstacleData[0].distance, obstacleData[0].depth); //printcur
	
}