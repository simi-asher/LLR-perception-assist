#include <stdio.h>
#include <conio.h>
#include <string>
#include <atlstr.h>
//#include <fstream>
//using std::ofstream;
//#include <iostream>
//using std::cout;
//using std::endl;
#define STRICT
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <ctype.h>


void system_error(char *name) {
	// Retrieve, format, and print out a message from the last error.  The 
	// `name' that's passed should be in the form of a present tense noun 
	// (phrase) such as "opening file".
	//
	char *ptr = NULL;
	FormatMessage(
		FORMAT_MESSAGE_ALLOCATE_BUFFER |
		FORMAT_MESSAGE_FROM_SYSTEM,
		0,
		GetLastError(),
		0,
		(char *)&ptr,
		1024,
		NULL);

	fprintf(stderr, "\nError %s: %s\n", name, ptr);
	LocalFree(ptr);
}

int Init_MPU(HANDLE *file, char *port_name) {
	//std::string str;
	
	COMMTIMEOUTS timeouts;
	DWORD written;
	DCB port;
	HANDLE keyboard = GetStdHandle(STD_INPUT_HANDLE);
	HANDLE screen = GetStdHandle(STD_OUTPUT_HANDLE);
	DWORD mode;
	//char port_name[128] = "\\\\.\\COM4";
	char init[] = ""; // e.g., "ATZ" to completely reset a modem.
	//const char * fname = "temp.txt";
	
	// hnd = CreateFileA(fname, GENERIC_WRITE, FILE_SHARE_DELETE | FILE_SHARE_READ | FILE_SHARE_WRITE,
	//	NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
	
	//if (argc > 2)
	//	sprintf_s(port_name, "\\\\.\\COM%c", argv[1][0]);

	// open the comm port.
	*file = CreateFile(port_name,
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		0,//synchronous mode
		NULL);

	if (INVALID_HANDLE_VALUE == *file) {
		system_error((char*)"opening file");
		return 1;
	}

	// get the current DCB, and adjust a few bits to our liking.
	memset(&port, 0, sizeof(port));
	port.DCBlength = sizeof(port);
	if (!GetCommState(*file, &port))
		system_error((char*)"getting comm state");
	if (!BuildCommDCB("baud=9600 parity=n data=8 stop=1", &port))
		system_error((char*)"building comm DCB");
	if (!SetCommState(*file, &port))
		system_error((char*)"adjusting port settings");

	// set short timeouts on the comm port.
	timeouts.ReadIntervalTimeout = 5;
	timeouts.ReadTotalTimeoutMultiplier = 5;
	timeouts.ReadTotalTimeoutConstant = 5;
	timeouts.WriteTotalTimeoutMultiplier = 5;
	timeouts.WriteTotalTimeoutConstant = 5;
	if (!SetCommTimeouts(*file, &timeouts))
		system_error((char*)"setting port time-outs.");

	// set keyboard to raw reading.
	if (!GetConsoleMode(keyboard, &mode))
		system_error((char*)"getting keyboard mode");
	mode &= ~ENABLE_PROCESSED_INPUT;
	if (!SetConsoleMode(keyboard, mode))
		system_error((char*)"setting keyboard mode");

	if (!EscapeCommFunction(*file, CLRDTR))
		system_error((char*)"clearing DTR");
	Sleep(200);
	if (!EscapeCommFunction(*file, SETDTR))
		system_error((char*)"setting DTR");

	if (!WriteFile(*file, init, sizeof(init), &written, NULL))
		system_error((char*)"writing data to port");

	if (written != sizeof(init))
		system_error((char*)"not all data written to port");

	return 0;
}

int Read_MPU(HANDLE *file, float *Y1,float *R, float *P2, float *Y2) { //simi removed float *P1,
	int ch = '0';
	double temp = 0.0;
	DWORD read;
	char buffer[1] = { NULL };
	int j = 0, k = 0, loopFlag = 0;
	
	CString str, holder;
	CString Sep = "/"; //data should be in format <Roll/Pitch1/Pitch2/Yaw>
	CString Token = "";
	int Pos = 0;
	PurgeComm(*file, PURGE_TXCLEAR | PURGE_RXCLEAR); //purging buffer
	holder.Empty();
	// basic terminal loop:
	while (loopFlag != 1)
	{
		// check for data on port and display it on screen.
		ReadFile(*file, buffer, sizeof(buffer), &read, NULL);

		//currently, data being read but along with junk chracters in between
		if (buffer[0] == '<') {
			holder.Empty();
			//i = 0;
			j = 1;

		}
		else if (buffer[0] == '>'&& j == 1) {
			k = 0;
			//std::cout << holder << endl;
			//i = 1;
			Pos = 0;
			Token = holder.Tokenize(Sep, Pos);
			while (Token != "")//if not try Pos!=-1
			{
				temp = std::stod({ Token.GetString(), static_cast<size_t>(Token.GetLength()) });
				switch(k) {
					case 0: *Y1 = float(temp);
							break;
					case 1: *R = float(temp);
							break;
					case 2: *P2 = float(temp);
							break;
					case 3: *Y2 = float(temp);
							break;
				}
				k++;
				if (k > 3)
					break;
				Token = holder.Tokenize(Sep, Pos);
			}
			//
			//while (k!=2 || Token != "") //&&Pos!=-1)
			//{
			//	//std::cout << Token<<endl;
			//	// Get next token.
			//	Token = holder.Tokenize(Sep, Pos);
			//	temp= std::stod({ Token.GetString(), static_cast<size_t>(Token.GetLength()) });
			//	*P1 = float(temp);
			//	Token = holder.Tokenize(Sep, Pos);
			//	if (k == 0 && Token != "") {
			//		temp = std::stod({ Token.GetString(), static_cast<size_t>(Token.GetLength()) });
			//		*P2 = float(temp);
			//		k = 1;
			//		Token = holder.Tokenize(Sep, Pos);
			//	}
			//	if (k == 1 && Token!="") {
			//		temp = std::stod({ Token.GetString(), static_cast<size_t>(Token.GetLength()) });
			//		*Y = float(temp);
			//		k = 2;
			//	}
			//}
			j = 0;
			//printf("RP1P2Y %f %f %f %f\n", *R,*P1, *P2, *Y);
			holder.Empty();
			loopFlag = 1;
		}
		//else if (i == 1 && buffer[0] != holder[holder.GetLength() - 1] && buffer[0] != ' ') {//this cmd doesnt prevent repeats but adding 10ms timeouts does
		//	holder += buffer[0];
		//}
		else if ( j==1 && (buffer[0] == '.' || buffer[0] == '/'|| buffer[0] == '-' || isdigit(buffer[0]))) { //i == 0 &&
			holder += buffer[0];
			//i = 1;
		}


		//// check for keypress, and write any out the port.
		//if (_kbhit()) {
		//	ch = _getch();
		//	//WriteFile(file, &ch, 1, &written, NULL);
		//}
		//// until user hits q
	} //while (loopFlag!=1); //ch != 'q'

	// close up and go home.
	//CloseHandle(keyboard);
	//CloseHandle(file);
	//CloseHandle(hnd);
	return 0;
}
