#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#pragma comment(lib, "user32.lib")

#include <bitset>
#include <random>
#include "stdafx.h"
#include <string.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <fstream>

#include <hidapi.h>

#include "packet.h"
#include "joycon.hpp"
#include "MouseController.hpp"
#include "tools.hpp"

// wxWidgets:
#include <wx/wx.h>
#include "MyApp.h"

#if defined(_WIN32)
#include <Windows.h>
#include <Lmcons.h>
#include <shlobj.h>
#endif

//#include "public.h"

// sio:
#include "sio_client.h"

//Vigem
#include <Xinput.h>
#include <ViGEm/Client.h>
#pragma comment(lib, "setupapi.lib")

//GLM - Used for the complementary filter (gyro/accel)
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>



#pragma warning(disable:4996)

#define JOYCON_VENDOR 0x057e
#define JOYCON_L_BT 0x2006
#define JOYCON_R_BT 0x2007
#define PRO_CONTROLLER 0x2009
#define JOYCON_CHARGING_GRIP 0x200e
#define SERIAL_LEN 18
#define PI 3.14159265359
#define L_OR_R(lr) (lr == 1 ? 'L' : (lr == 2 ? 'R' : '?'))


std::vector<Joycon> joycons;
MouseController MC;
unsigned char buf[65];
int res = 0;

// sio:
sio::client myClient;

int RingconStatus = 0;

// this is awful, don't do this:
wxStaticText* gyroComboCodeText;
void setGyroComboCodeText(int code);

wxCheckBox* gyroCheckBox;

int subloop = 0;

int Ringcon = 0x0A;
double Ringcon_detail = 0x00;
double Ringcon_Sum = 0;
double prevRingcon_Sum = 0;
int prevRingcon = 0x0A;
int ringconcounter = 0;

#define runarraylength 50
int runningindex[runarraylength] = { 0 };
int runvalue = 0;
int squatvalue = 0;
bool running = false;
bool ringconattached = false;
bool squatting = false;

bool first = true;
int initTimes = 0;
float initValue = 0;
bool defaultValueInited = false;

float validroll = 0.00;
float validpitch = 0.00;

bool leftmousedown = false;
bool rightmousedown = false;
float squatmousemult = 1; //Slow down mouse movement depending on how far you squat. Makes Ringcon clicking easier.

bool p1ready = false;
bool p2ready = false;

//Init VigEm
const auto client = vigem_alloc();
const auto retval = vigem_connect(client);
PVIGEM_TARGET pad1 = 0;
PVIGEM_TARGET pad2 = 0;
XUSB_REPORT report;
WORD remappedbtnsr=0;
WORD remappedbtnsl=0;
BYTE RightTrigger = 0;
BYTE LeftTrigger = 0;
int MaxStick = 32767;

LONG sThumbLX = 0;
LONG sThumbLY = 0;
LONG sThumbRX = 0;
LONG sThumbRY = 0;

struct Settings {

	// Enabling this combines both JoyCons to a single Vigem Device
	// when combineJoyCons == false:
	// JoyCon(L) is mapped to Vigem Device #1
	// JoyCon(R) is mapped to Vigem Device #2
	// when combineJoyCons == true:
	// JoyCon(L) and JoyCon(R) are mapped to Vigem Device #1
	bool combineJoyCons = false;

	bool reverseX = false;// reverses joystick x (both sticks)
	bool reverseY = false;// reverses joystick y (both sticks)

	bool usingGrip = false;
	bool usingBluetooth = true;
	bool disconnect = false;

	// enables motion controls
	bool enableGyro = false;
	bool squatSlowsMouse = false;

	// gyroscope (mouse) sensitivity:
	float gyroSensitivityX = 150.0f;
	float gyroSensitivityY = 150.0f;


	// prefer the left joycon for gyro controls
	bool preferLeftJoyCon = false;

	// combo code to set key combination to disable gyroscope for quick turning in games. -1 to disable.
	int gyroscopeComboCode = 4;

	// toggles between two different toggle types
	// disabled = traditional toggle
	// enabled = while button(s) are held gyro is enabled
	bool quickToggleGyro = false;

	// inverts the above function
	bool invertQuickToggle = false;

	// for dolphin, mostly:
	//bool dolphinPointerMode = false;

	// so that you don't rapidly toggle the gyro controls every frame:
	bool canToggleGyro = true;

	// plays a version of the mario theme by vibrating
	// the first JoyCon connected.
	bool marioTheme = false;

	// bool to restart the program
	bool restart = false;

	// auto start the program
	bool autoStart = false;

	// Run presses button
	bool Runpressesbutton = false;

	// Some Ringcons do not default to 10 when not being flexed. This is a manual adjustment to make it happen:
	float RingconFix = 0.0f;

	// debug file:
	FILE* outputFile;

	// Use Ringcon with full set of buttons, right handed::
	bool RingconFullRH = false;
	// where to connect:
	std::string host = "";
	// string to send:
	std::string controllerState = "";
	// Use Ringcon with full set of buttons, left handed:
	bool RingconToAnalog = false;


	bool RingconToLeftAnalog = false;

	// poll options:

	// running unlocks the gyro:
	bool rununlocksgyro = false;

	// times to poll per second per joycon:
	float pollsPerSec = 30.0f;
	//float pollsPerSec = 100.0f;
	//float pollsPerSec = 200.0f;

	// time to sleep (in ms) between polls:
	float timeToSleepMS = 1.0f;

	float RingconReferenceValue = 11.5f;

	float ringconPushMul = 1.0f;
	float ringconPullMul = 1.0f;

	// version number
	std::string version = "1.04";

} settings;


struct Tracker {

	int var1 = 0;
	int var2 = 0;
	int counter1 = 0;

	float low_freq = 200.0f;
	float high_freq = 500.0f;

	float relX = 0;
	float relY = 0;

	float anglex = 0;
	float angley = 0;
	float anglez = 0;

	glm::fquat quat = glm::angleAxis(0.0f, glm::vec3(1.0, 0.0, 0.0));
	// get current time
	//std::chrono::high_resolution_clock tNow;
	//std::chrono::steady_clock::time_point tPoll = std::chrono::high_resolution_clock::now();
	std::vector<std::chrono::steady_clock::time_point> tPolls;
	//Tracker(int value) : tPolls(100, std::chrono::high_resolution_clock::now()) {}
	//auto tSleepStart = std::chrono::high_resolution_clock::now();

	float previousPitch = 0;
} tracker;




void handle_input(Joycon* jc, uint8_t* packet, int len) {

	// bluetooth button pressed packet:
	if (packet[0] == 0x3F) {

		uint16_t old_buttons = jc->buttons;
		int8_t old_dstick = jc->dstick;

		jc->dstick = packet[3];
		// todo: get button states here aswell:
	}

	// input update packet:
	// 0x21 is just buttons, 0x30 includes gyro, 0x31 includes NFC (large packet size)
	if (packet[0] == 0x30 || packet[0] == 0x31 || packet[0] == 0x32) {

		// offset for usb or bluetooth data:
		/*int offset = settings.usingBluetooth ? 0 : 10;*/
		int offset = jc->bluetooth ? 0 : 10;

		uint8_t* btn_data = packet + offset + 3;

		// get button states:
		{
			uint16_t states = 0;
			uint16_t states2 = 0;

			// Left JoyCon:
			if (jc->left_right == 1) {
				states = (btn_data[1] << 8) | (btn_data[2] & 0xFF);
				// Right JoyCon:
			}
			else if (jc->left_right == 2) {
				states = (btn_data[1] << 8) | (btn_data[0] & 0xFF);
				// Pro Controller:
			}
			else if (jc->left_right == 3) {
				states = (btn_data[1] << 8) | (btn_data[2] & 0xFF);
				states2 = (btn_data[1] << 8) | (btn_data[0] & 0xFF);
			}

			jc->buttons = states;
			// Pro Controller:
			if (jc->left_right == 3) {
				jc->buttons2 = states2;

				// fix some non-sense the Pro Controller does
				// clear nth bit
				//num &= ~(1UL << n);
				jc->buttons &= ~(1UL << 9);
				jc->buttons &= ~(1UL << 10);
				jc->buttons &= ~(1UL << 12);
				jc->buttons &= ~(1UL << 14);

				jc->buttons2 &= ~(1UL << 8);
				jc->buttons2 &= ~(1UL << 11);
				jc->buttons2 &= ~(1UL << 13);
			}
		}

		// get stick data:
		uint8_t* stick_data = packet + offset;
		if (jc->left_right == 1) {
			stick_data += 6;
		}
		else if (jc->left_right == 2) {
			stick_data += 9;
		}

		uint16_t stick_x = stick_data[0] | ((stick_data[1] & 0xF) << 8);
		uint16_t stick_y = (stick_data[1] >> 4) | (stick_data[2] << 4);
		jc->stick.x = stick_x;
		jc->stick.y = stick_y;

		// use calibration data:
		jc->CalcAnalogStick();

		// pro controller:
		if (jc->left_right == 3) {
			stick_data += 6;
			uint16_t stick_x = stick_data[0] | ((stick_data[1] & 0xF) << 8);
			uint16_t stick_y = (stick_data[1] >> 4) | (stick_data[2] << 4);
			jc->stick.x = (int)(unsigned int)stick_x;
			jc->stick.y = (int)(unsigned int)stick_y;
			stick_data += 3;
			uint16_t stick_x2 = stick_data[0] | ((stick_data[1] & 0xF) << 8);
			uint16_t stick_y2 = (stick_data[1] >> 4) | (stick_data[2] << 4);
			jc->stick2.x = (int)(unsigned int)stick_x2;
			jc->stick2.y = (int)(unsigned int)stick_y2;

			// calibration data:
			jc->CalcAnalogStick();
		}

		jc->battery = (stick_data[1] & 0xF0) >> 4;
		//printf("JoyCon battery: %d\n", jc->battery);

		// Accelerometer:
		// Accelerometer data is absolute (m/s^2)
		{

			// get accelerometer X:
			jc->accel.x = (float)(uint16_to_int16(packet[13] | (packet[14] << 8) & 0xFF00)) * jc->acc_cal_coeff[0];

			// get accelerometer Y:
			jc->accel.y = (float)(uint16_to_int16(packet[15] | (packet[16] << 8) & 0xFF00)) * jc->acc_cal_coeff[1];

			// get accelerometer Z:
			jc->accel.z = (float)(uint16_to_int16(packet[17] | (packet[18] << 8) & 0xFF00)) * jc->acc_cal_coeff[2];
		}



		// Gyroscope:
		// Gyroscope data is relative (rads/s)
		if (jc->left_right == 2 && ringconattached) {
			{

				// get roll:
				jc->gyro.roll = (float)((uint16_to_int16(packet[35] | (packet[36] << 8) & 0xFF00)) - jc->sensor_cal[1][0]) * jc->gyro_cal_coeff[0]; //23 24 was working

				// get pitch:
				jc->gyro.pitch = (float)((uint16_to_int16(packet[31] | (packet[32] << 8) & 0xFF00)) - jc->sensor_cal[1][1]) * jc->gyro_cal_coeff[1]; // 19 20 was working

				// get yaw:
				jc->gyro.yaw = (float)((uint16_to_int16(packet[33] | (packet[34] << 8) & 0xFF00)) - jc->sensor_cal[1][2]) * jc->gyro_cal_coeff[2]; // 21 22 was working

				// Note: All of the below orientations are from the point of view of the ringcon. May not line up with official terminology.
				//13-14 Roll
				//15-16 Pitch centred at horizontal
				//17-18 Pitch centred at vertical
				//19-20 Gyro pitch - Forward = +, Backward = -
				//21-22 Gyro yaw (needed for running) - When running, stepping down = +, stepping up = -
				//23-24 Gyro roll - Clockwise = +, Anticlockwise = -
				//25-26 Roll anticlockwise +, clockwise -
				//27-28 Pitch centred at horizontal - up = -, down = +
				//29-30 Pitch centred at vertical - up = -, down = +
				//31-32, 33-34, 35-36 arebouncing around but have something to do with the gyro. maybe i need a single byte?
				//printf("%f      %f     %f", jc->gyro.roll, jc->gyro.yaw, jc->gyro.pitch);
			}
		}
		else {

			// get roll:
			jc->gyro.roll = (float)((uint16_to_int16(packet[19] | (packet[20] << 8) & 0xFF00)) - jc->sensor_cal[1][0]) * jc->gyro_cal_coeff[0]; //23 24 was working, now not so much

			// get pitch:
			jc->gyro.pitch = (float)((uint16_to_int16(packet[21] | (packet[22] << 8) & 0xFF00)) - jc->sensor_cal[1][1]) * jc->gyro_cal_coeff[1]; // 19 20 was working

			// get yaw:
			jc->gyro.yaw = (float)((uint16_to_int16(packet[23] | (packet[24] << 8) & 0xFF00)) - jc->sensor_cal[1][2]) * jc->gyro_cal_coeff[2]; // 21 22 was working
		}

		// offsets:
		{
			jc->setGyroOffsets();

			jc->gyro.roll -= jc->gyro.offset.roll;
			jc->gyro.pitch -= jc->gyro.offset.pitch;
			jc->gyro.yaw -= jc->gyro.offset.yaw;
		}

	}






	// handle button combos:
	{
		bool lightpress = false;
		bool lightpull = false;
		bool heavypress = false;
		bool heavypull = false;

		// right:
		if (jc->left_right == 2 && ringconattached) {
			//Ringcon logic - Default values - int prevringcon = 0x0A; int ringconcounter = 0;
			if (packet[0] != 48)
				return;

			if (first)
			{
				first = false;
				printf("成功读取到环数据\n");
				RingconStatus = 1;
			}

			Ringcon = packet[40];
			Ringcon_detail = packet[39];


			//if (Ringcon == 0x00) { //The Ringcon reading has started randomly putting zero in to the reading, I must not be initializing it properly. This is a hack to get around that.
			//	Ringcon = prevRingcon;
			//}

			int temp = settings.RingconFix;
			float fixF = settings.RingconFix - temp;
			Ringcon = Ringcon + temp;

			if (Ringcon >= 100) {
				Ringcon = Ringcon - 255;
			}

			float detail = Ringcon_detail / 255.0;
			int temp_detail = detail * 100;
			int temp_detail_1000 = detail * 1000 - temp_detail * 10;
			if (temp_detail_1000 >= 5)
			{
				temp_detail++;
			}
			detail = (temp_detail) / 100.0;

			
			Ringcon_Sum = Ringcon + detail + fixF;

			if (!defaultValueInited)
			{
				if (initTimes == 0)
				{
					initValue = Ringcon_Sum;
					printf("正在校准环默认值，请静置环...\n");
					RingconStatus = 2;
				}
				if (initTimes == 250)//50次环数据无大波动，则认为是校准值
				{
					defaultValueInited = true;
					int temp_initValue = initValue * 100;
					initValue = temp_initValue / 100.0;
					settings.RingconReferenceValue = initValue;
					printf("\n校准成功 (%.2f)\n", initValue);
					RingconStatus = 3;
				}
				else
				{
					float del = initValue - Ringcon_Sum;
					if (del > 0.15 || del < -0.15)//波动大于0.15时则重新计算
					{
						initTimes = 0;
						initValue = Ringcon_Sum;
					}
					else
					{
						initTimes++;
						initValue = (Ringcon_Sum + initValue) / 2;
						if (initTimes % 10 == 0)
						{
							printf(".");
						}
					}
				}
				//initTimes++;
			}
			else
			{
				float ringconMul = settings.ringconPushMul;
				if (Ringcon_Sum < settings.RingconReferenceValue)
					ringconMul = settings.ringconPullMul;

				Ringcon_Sum = settings.RingconReferenceValue + (Ringcon_Sum - settings.RingconReferenceValue) * ringconMul;
			}
			
			if (Ringcon_Sum != prevRingcon_Sum && defaultValueInited) {

				printf("%.2f\n", Ringcon_Sum - settings.RingconReferenceValue);
			}
			
			if (settings.RingconFullRH) { //The sensor readings change if it is being held sideways
				if (Ringcon == 0x0A || Ringcon == 0x09 || Ringcon == 0x08 || Ringcon == 0x07) { //Deadzone
					ringconcounter = 0;
				}

				if (Ringcon == 0x01 || Ringcon == 0xFF || Ringcon == 0xFE) {
					heavypress = false; //turn off heavy press, may damage Ringcon as it goes outside the flex range
					//ringconcounter = -1;
				}
				if (Ringcon == 0x0D || Ringcon == 0x0E || Ringcon == 0x0F) {
					heavypull = true;
					ringconcounter = -1;
				}
				if (Ringcon >= 0x02 && Ringcon <= 0x06 && ringconcounter != -1) {
					/*if (Ringcon < prevringcon && ringconcounter < 10) {
						ringconcounter = 0;
					}
					else if (Ringcon == prevringcon && ringconcounter < 10) {
						ringconcounter++;
					}
					else {*/
					lightpress = true;
					ringconcounter = 20;
					//}
				}
				if (Ringcon <= 0x0C && Ringcon >= 0x0B && ringconcounter != -1) {
					if (Ringcon > prevRingcon && ringconcounter < 10) {
						ringconcounter = 0;
					}
					else if (Ringcon == prevRingcon && ringconcounter < 10) {
						ringconcounter++;
					}
					else {
						lightpull = true;
						ringconcounter = 20;
					}
				}
			}
			else {
				if (Ringcon == 0x0A || Ringcon == 0x09 || Ringcon == 0x08 || Ringcon == 0x0B) { //Deadzone
					ringconcounter = 0;
				}

				if (Ringcon >= 0x11) {
					heavypress = true;
					ringconcounter = -1;
				}
				if (Ringcon <= 0x03 && Ringcon != 0x00) {
					heavypull = true;
					ringconcounter = -1;
				}
				if (Ringcon >= 0x0C && Ringcon <= 0x10 && ringconcounter != -1) {
					if (Ringcon > prevRingcon && ringconcounter < 10) {
						ringconcounter = 0;
					}
					else if (Ringcon == prevRingcon && ringconcounter < 10) {
						ringconcounter++;
					}
					else {
						lightpress = true;
						ringconcounter = 20;
					}
				}
				if (Ringcon <= 0x07 && Ringcon >= 0x04 && ringconcounter != -1) {
					if (Ringcon < prevRingcon && ringconcounter < 10) {
						ringconcounter = 0;
					}
					else if (Ringcon == prevRingcon && ringconcounter < 10) {
						ringconcounter++;
					}
					else {
						lightpull = true;
						ringconcounter = 20;
					}
				}
			}

			prevRingcon = Ringcon;
			prevRingcon_Sum = Ringcon_Sum;
			//printf("%i \n\n", Ringcon);
		}

		// left:
		if (jc->left_right == 1) {

			// Determine whether the left joycon is telling us we are running
			runningindex[runvalue % runarraylength] = jc->gyro.pitch;
			runvalue++;
			int sum = 0;
			int average = 0;
			for (int i = 0; i < runarraylength; i++) {
				if (runningindex[i] >= 0) {
					sum += (runningindex[i] * 2);
				}
				else {
					sum -= (runningindex[i] * 2); //Too many zeros means the average will be 0 even when there are quite a lot of numbers with values. This seems to be a good value with arraylength at 50.
				}
			}

			average = sum / runarraylength;

			//printf("%i\n", average); //walk 0-1, jog 1-2, run 2-3, sprint 3-4
			if (average > 0) {
				running = true;
				if (settings.Runpressesbutton) {
					jc->buttons |= 1U << 4; //sr = run
				}
			}
			else {
				running = false;
			}

			//jc->btns.sl = (jc->buttons & (1 << 5)) ? 1 : 0; // set a bit: *ptr |= 1 << index;
			//sprint button
			if (average >= 3) { //sprint
				jc->buttons |= 1U << 5; //sl = sprint
			}
			//int squatvalue = 0;
			//printf("%f", jc->accel.z); //9.8 when horizontal. 0 when vertical. Goes to minus when facing down or backwards.
			if (jc->accel.z > 6.0 && jc->accel.z < 12.0) {
				squatvalue++;
				if (squatvalue >= 20 && !settings.squatSlowsMouse) {
					jc->buttons |= 1U << 8; //jc->btns.minus = (jc->buttons & (1 << 8)) ? 1 : 0;
				}
			}
			else {
				squatvalue = 0;
			}

			if (jc->accel.z > 2.0 && jc->accel.z < 12.0) {
				squatting = true;
			}
			else {
				squatting = false;
			}

			if (settings.squatSlowsMouse && !running) {
				if (jc->accel.z <= 0.1) {
					squatmousemult = 1;
				}
				else if (jc->accel.z >= 9.0) {
					squatmousemult = 0.1;
				}
				else {
					squatmousemult = 1 - (jc->accel.z * 0.1);
				}
			}
			else {
				squatmousemult = 1;
			}

			//Mouse buttons
			if (settings.enableGyro && !settings.combineJoyCons) {
				if (jc->buttons & (1 << 7) && !leftmousedown) { //ZL controls left mouse button
					MC.LeftClickDown();
					leftmousedown = true;
				}
				if (!(jc->buttons & (1 << 7)) && leftmousedown) { //ZL controls left mouse button
					MC.LeftClickUp();
					leftmousedown = false;
				}
				if (jc->buttons & (1 << 6) && !rightmousedown) { //L controls right mouse button
					MC.RightClickDown();
					rightmousedown = true;
				}
				if (!(jc->buttons & (1 << 6)) && rightmousedown) { //L controls right mouse button
					MC.RightClickUp();
					rightmousedown = false;
				}
			}

			jc->btns.down = (jc->buttons & (1 << 0)) ? 1 : 0;
			jc->btns.up = (jc->buttons & (1 << 1)) ? 1 : 0;
			jc->btns.right = (jc->buttons & (1 << 2)) ? 1 : 0;
			jc->btns.left = (jc->buttons & (1 << 3)) ? 1 : 0;
			jc->btns.sr = (jc->buttons & (1 << 4)) ? 1 : 0;
			jc->btns.sl = (jc->buttons & (1 << 5)) ? 1 : 0;
			jc->btns.l = (jc->buttons & (1 << 6)) ? 1 : 0;
			jc->btns.zl = (jc->buttons & (1 << 7)) ? 1 : 0;
			jc->btns.minus = (jc->buttons & (1 << 8)) ? 1 : 0;
			jc->btns.stick_button = (jc->buttons & (1 << 11)) ? 1 : 0;
			jc->btns.capture = (jc->buttons & (1 << 13)) ? 1 : 0;
		}

		// right:
		if (jc->left_right == 2) {

			//Ringcon stuff

			if (lightpress == true) {
				jc->buttons |= 1U << 4;
			}

			if (heavypress == true) {
				jc->buttons |= 1U << 7;
			}

			if (lightpull == true) {
				jc->buttons |= 1U << 5;
			}

			if (heavypull == true) {
				jc->buttons |= 1U << 6;
			}

			//Mouse buttons
			//printf("%i\n", Ringcon);
			if (settings.enableGyro) {
				if ((jc->buttons & (1 << 7) || Ringcon >= 0x0C) && !leftmousedown) { //ZR controls left mouse button
					MC.LeftClickDown();
					leftmousedown = true;
				}
				if (!(jc->buttons & (1 << 7) || Ringcon >= 0x0C) && leftmousedown) {
					MC.LeftClickUp();
					leftmousedown = false;
				}
				if ((jc->buttons & (1 << 6) || lightpull || heavypull) && !rightmousedown) { //R controls right mouse button
					MC.RightClickDown();
					rightmousedown = true;
				}
				if (!(jc->buttons & (1 << 6) || lightpull || heavypull) && rightmousedown) {
					MC.RightClickUp();
					rightmousedown = false;
				}
			}

			jc->btns.y = (jc->buttons & (1 << 0)) ? 1 : 0;
			jc->btns.x = (jc->buttons & (1 << 1)) ? 1 : 0;
			jc->btns.b = (jc->buttons & (1 << 2)) ? 1 : 0;
			jc->btns.a = (jc->buttons & (1 << 3)) ? 1 : 0;
			jc->btns.sr = (jc->buttons & (1 << 4)) ? 1 : 0;
			jc->btns.sl = (jc->buttons & (1 << 5)) ? 1 : 0;
			jc->btns.r = (jc->buttons & (1 << 6)) ? 1 : 0;
			jc->btns.zr = (jc->buttons & (1 << 7)) ? 1 : 0;
			jc->btns.plus = (jc->buttons & (1 << 9)) ? 1 : 0;
			jc->btns.stick_button = (jc->buttons & (1 << 10)) ? 1 : 0;
			jc->btns.home = (jc->buttons & (1 << 12)) ? 1 : 0;
		}

		// pro controller:
		if (jc->left_right == 3) {

			// left:
			jc->btns.down = (jc->buttons & (1 << 0)) ? 1 : 0;
			jc->btns.up = (jc->buttons & (1 << 1)) ? 1 : 0;
			jc->btns.right = (jc->buttons & (1 << 2)) ? 1 : 0;
			jc->btns.left = (jc->buttons & (1 << 3)) ? 1 : 0;
			jc->btns.sr = (jc->buttons & (1 << 4)) ? 1 : 0;
			jc->btns.sl = (jc->buttons & (1 << 5)) ? 1 : 0;
			jc->btns.l = (jc->buttons & (1 << 6)) ? 1 : 0;
			jc->btns.zl = (jc->buttons & (1 << 7)) ? 1 : 0;
			jc->btns.minus = (jc->buttons & (1 << 8)) ? 1 : 0;
			jc->btns.stick_button = (jc->buttons & (1 << 11)) ? 1 : 0;
			jc->btns.capture = (jc->buttons & (1 << 13)) ? 1 : 0;

			// right:
			jc->btns.y = (jc->buttons2 & (1 << 0)) ? 1 : 0;
			jc->btns.x = (jc->buttons2 & (1 << 1)) ? 1 : 0;
			jc->btns.b = (jc->buttons2 & (1 << 2)) ? 1 : 0;
			jc->btns.a = (jc->buttons2 & (1 << 3)) ? 1 : 0;
			jc->btns.sr = (jc->buttons2 & (1 << 4)) ? 1 : 0;
			jc->btns.sl = (jc->buttons2 & (1 << 5)) ? 1 : 0;
			jc->btns.r = (jc->buttons2 & (1 << 6)) ? 1 : 0;
			jc->btns.zr = (jc->buttons2 & (1 << 7)) ? 1 : 0;
			jc->btns.plus = (jc->buttons2 & (1 << 9)) ? 1 : 0;
			jc->btns.stick_button2 = (jc->buttons2 & (1 << 10)) ? 1 : 0;
			jc->btns.home = (jc->buttons2 & (1 << 12)) ? 1 : 0;

		}
	}
}


void updateVigEmDevice2(Joycon* jc) { 

	UINT DevID;

	PVOID pPositionMessage;
	// HID_DEVICE_ATTRIBUTES attrib;
	BYTE id = 1;

	// Set destination Vigem device
	DevID = jc->VigemNumber;
	id = (BYTE)DevID;

	// Set Stick data
	if (!settings.combineJoyCons) {
		sThumbLX = MaxStick * (jc->stick.CalX);
		sThumbLY = MaxStick * (jc->stick.CalY);
	}
	if (jc->left_right== 2) {
		if (settings.RingconFullRH) {
			sThumbRX = -MaxStick * (jc->stick.CalX);
			sThumbRY = -MaxStick * (jc->stick.CalY);
		}
		else {
			sThumbRY = -MaxStick * (jc->stick.CalX);
			sThumbRX = MaxStick * (jc->stick.CalY);
		}
	}
	// pro controller:
	if (jc->left_right == 3) {
		sThumbLX = MaxStick * (jc->stick.CalX);
		sThumbLY = MaxStick * (jc->stick.CalY);
		sThumbRX = MaxStick * (jc->stick2.CalX);
		sThumbRY = MaxStick * (jc->stick2.CalY);
	}


	// prefer left joycon for gyroscope controls:
	int a = -1;
	int b = -1;
	if (settings.preferLeftJoyCon) {
		a = 1;
		b = 2;
	}
	else {
		a = 2;
		b = 1;
	}

	bool gyroComboCodePressed = false;



	// gyro / accelerometer data: 
	if ((((jc->left_right == a) || (jc->left_right == 3)) && settings.combineJoyCons) || !settings.combineJoyCons) { //|| (joycons.size() == 1 && jc->left_right == b)

		int multiplier;


		// Gyroscope (roll, pitch, yaw):
		multiplier = 1000;




		// complementary filtered tracking
		// uses gyro + accelerometer

		// set to 0:

		//float gyroCoeff = .78; // Originally 0.001
		float gyroCoeff = .78;

		//Pitchdegrees accell = (glm::degrees((atan2(direction of pitch roll, - direction of gravity) + PI))) - 180;

		// x:
		float pitchDegreesAccel;
		if (settings.RingconFullRH) {
			pitchDegreesAccel = (glm::degrees((atan2(jc->accel.y, -jc->accel.x) + PI))) - 180;
		}
		else {
			pitchDegreesAccel = (glm::degrees((atan2(-jc->accel.z, -jc->accel.y) + PI))) - 180;
		}

		float pitchDegreesGyro = -jc->gyro.pitch * gyroCoeff;
		float pitch = 0;
		float pitchmult = 0;

		tracker.anglex += pitchDegreesGyro; //~Max of 10 each way

		pitchmult = abs(pitchDegreesGyro * 0.004) + 0.005; // The multiplier for the accelerometer. The more the gyro moves, the higher this is.

		if (pitchDegreesAccel > 180) {
			pitchDegreesAccel = 180;
		}
		else if (pitchDegreesAccel < -180) {
			pitchDegreesAccel = -180;
		}

		if ((pitchDegreesAccel - tracker.anglex) > 360) {
			tracker.anglex += 360;
		}
		else if ((tracker.anglex - pitchDegreesAccel) > 360) {
			tracker.anglex -= 360;
		}
		tracker.anglex = (tracker.anglex * (1 - pitchmult)) + (pitchDegreesAccel * pitchmult); //multiplers need to add to 1
		pitch = tracker.anglex;

		glm::fquat delx = glm::angleAxis(glm::radians(pitch), glm::vec3(1.0, 0.0, 0.0));
		tracker.quat = tracker.quat * delx;

		//printf("%f   ", pitchmult); 

		// y:
		float rollDegreesAccel;
		if (settings.RingconFullRH) {
			rollDegreesAccel = (glm::degrees((atan2(jc->accel.y, -jc->accel.x) + PI))) - 180;
		}
		else {
			rollDegreesAccel = (glm::degrees((atan2(-jc->accel.x, -jc->accel.y) + PI))) - 180;
		}

		float rollDegreesGyro = jc->gyro.roll * gyroCoeff;
		float roll = 0;
		float rollmult = 0;

		tracker.angley += rollDegreesGyro;
		rollmult = abs(rollDegreesGyro * 0.004) + 0.005;

		if (rollDegreesAccel > 180) {
			rollDegreesAccel = 180;
		}
		else if (rollDegreesAccel < -180) {
			rollDegreesAccel = -180;
		}

		if ((rollDegreesAccel - tracker.angley) > 360) {
			tracker.angley += 360;
		}
		else if ((tracker.angley - rollDegreesAccel) > 360) {
			tracker.angley -= 360;
		}

		tracker.angley = (tracker.angley * (1 - rollmult)) + (rollDegreesAccel * rollmult);//0.98 0.02
		//tracker.angley = -rollInDegreesAccel;
		roll = tracker.angley;


		//glm::fquat dely = glm::angleAxis(glm::radians(roll), glm::vec3(0.0, 0.0, 1.0));
		//tracker.quat = tracker.quat*dely;

		//printf("%f   \n\n", roll);



		// z:
		float yawDegreesAccel = (glm::degrees((atan2(-jc->accel.z, -jc->accel.x) + PI))) - 180;
		float yawDegreesGyro = jc->gyro.yaw * gyroCoeff;
		float yaw = 0;
		float yawmult = 0;

		//printf("%f     ", yawDegreesAccel);

		tracker.anglez += yawDegreesGyro;
		yawmult = abs(yawDegreesGyro * 0.004) + 0.005;

		if (yawDegreesAccel > 180) {
			yawDegreesAccel = 180;
		}
		else if (yawDegreesAccel < -180) {
			yawDegreesAccel = -180;
		}

		if ((yawDegreesAccel - tracker.anglez) > 360) {
			tracker.anglez += 360;
		}
		else if ((tracker.anglez - yawDegreesAccel) > 360) {
			tracker.anglez -= 360;
		}

		tracker.anglez = (tracker.anglez * (1 - yawmult)) + (yawDegreesAccel * yawmult);//0.98 0.02
		//tracker.angley = -rollInDegreesAccel;
		yaw = tracker.anglez;

		//printf("%f   \n\n", yaw); //Good for Ringcon sideways, right handed. 

		//printf("%.0f      %.0f       %.0f      %.0i     \n\n", jc->gyro.roll, jc->gyro.pitch, jc->gyro.yaw, Ringcon);

		float relX2 = -jc->gyro.yaw * settings.gyroSensitivityX * squatmousemult;
		float relY2 = jc->gyro.pitch * settings.gyroSensitivityY * squatmousemult;

		relX2 /= 10;
		relY2 /= 10;

		//printf("%.3f %.3f %.3f\n", abs(jc->gyro.roll), abs(jc->gyro.pitch), abs(jc->gyro.yaw));
		//printf("%.2f %.2f\n", relX2, relY2);

		// check if combo keys are pressed:
		int comboCodeButtons = -1;
		if (jc->left_right != 3) {
			comboCodeButtons = jc->buttons;
		}
		else {
			comboCodeButtons = ((uint32_t)jc->buttons2 << 16) | jc->buttons;
		}

		//因修改为dll此处刷新界面数据注释
		//setGyroComboCodeText(comboCodeButtons);
		if (comboCodeButtons == settings.gyroscopeComboCode) {
			gyroComboCodePressed = true;
		}
		else {
			gyroComboCodePressed = false;
		}

		if (!gyroComboCodePressed) {
			settings.canToggleGyro = true;
		}

		if (settings.canToggleGyro && gyroComboCodePressed && !settings.quickToggleGyro) {
			settings.enableGyro = !settings.enableGyro;
			gyroCheckBox->SetValue(settings.enableGyro);
			settings.canToggleGyro = false;
		}

		if (jc->left_right == 2) {
			relX2 *= -1;
			relY2 *= -1;
		}

		bool gyroActuallyOn = false;

		if (settings.enableGyro && settings.quickToggleGyro) {
			// check if combo keys are pressed:
			if (settings.invertQuickToggle) {
				if (!gyroComboCodePressed) {
					gyroActuallyOn = true;
				}
			}
			else {
				if (gyroComboCodePressed) {
					gyroActuallyOn = true;
				}
			}
		}

		if (settings.enableGyro && !settings.quickToggleGyro) {
			gyroActuallyOn = true;
		}

		float mult = settings.gyroSensitivityX * 10.0f;
		int joymult = 1100; // ~32767/30 - gyro is in degrees, max forward should be 30ish degrees 1100

		if (gyroActuallyOn) {
			MC.moveRel3(relX2, relY2);
		}

		if (!running && settings.rununlocksgyro) {
			if (settings.squatSlowsMouse && squatting) {
				//Ignore the run unlocks gyro
			}
			else {
				mult = 0;
				joymult = 0;
			}
		}

		if ((pitch > -5) && (pitch < 5) && running) {
			pitch = 0; //Deadzone
		}

		if ((roll > -5) && (roll < 5) && running) {
			roll = 0; //Deadzone
		}

		if (settings.RingconToAnalog) {

			if (settings.RingconToLeftAnalog)
			{
				sThumbLY = ((settings.RingconReferenceValue - Ringcon_Sum) * 3276);
			}
			else
			{
				sThumbRY = ((settings.RingconReferenceValue - Ringcon_Sum) * 3276);
			}

			//sThumbRY = ((Ringcon - 10) * 3276);
			//printf("rowData：Ringcon:%i  LY:%i\n", Ringcon, sThumbRY);
			//sThumbRY = Ringcon;
			
		}

		if (Ringcon == 0x0A || Ringcon == 0x09 || Ringcon == 0x08 || Ringcon == 0x0B) { //Deadzone
			Ringcon = 10;
		}

		if (settings.combineJoyCons) {
			if (ringconattached) {
				sThumbLX = (roll * joymult * squatmousemult);
				if (settings.RingconFullRH) {
					sThumbLY = (yaw * joymult * squatmousemult);
				}
				else {
					sThumbLY = (pitch * joymult * squatmousemult);
				}
			}
			else {
				sThumbLX = (yaw * joymult * squatmousemult);
				if (settings.RingconFullRH) {
					sThumbLY = (pitch * joymult * squatmousemult);
				}
				else {
					sThumbLY = (roll * joymult * squatmousemult);
				}
				//report.wSlider = 16384 + ((Ringcon - 10) * 1640); No space on the controller for this. This is the analog version of the Ringcon.
			}
		}
	}

	// Set button data

	/*	For reference
	// left:
	jc->btns.down = (jc->buttons & (1 << 0)) ? 1 : 0;
	jc->btns.up = (jc->buttons & (1 << 1)) ? 1 : 0;
	jc->btns.right = (jc->buttons & (1 << 2)) ? 1 : 0;
	jc->btns.left = (jc->buttons & (1 << 3)) ? 1 : 0;
	jc->btns.sr = (jc->buttons & (1 << 4)) ? 1 : 0; //Run
	jc->btns.sl = (jc->buttons & (1 << 5)) ? 1 : 0; //Sprint
	jc->btns.l = (jc->buttons & (1 << 6)) ? 1 : 0;
	jc->btns.zl = (jc->buttons & (1 << 7)) ? 1 : 0;
	jc->btns.minus = (jc->buttons & (1 << 8)) ? 1 : 0; //Squat
	jc->btns.stick_button = (jc->buttons & (1 << 11)) ? 1 : 0;
	jc->btns.capture = (jc->buttons & (1 << 13)) ? 1 : 0;

	// right:
	jc->btns.y = (jc->buttons2 & (1 << 0)) ? 1 : 0;
	jc->btns.x = (jc->buttons2 & (1 << 1)) ? 1 : 0;
	jc->btns.b = (jc->buttons2 & (1 << 2)) ? 1 : 0;
	jc->btns.a = (jc->buttons2 & (1 << 3)) ? 1 : 0;
	jc->btns.sr = (jc->buttons2 & (1 << 4)) ? 1 : 0; //Lightpull
	jc->btns.sl = (jc->buttons2 & (1 << 5)) ? 1 : 0; //Lightpress
	jc->btns.r = (jc->buttons2 & (1 << 6)) ? 1 : 0; //Heavypress
	jc->btns.zr = (jc->buttons2 & (1 << 7)) ? 1 : 0; //Heavypull
	jc->btns.plus = (jc->buttons2 & (1 << 9)) ? 1 : 0;
	jc->btns.stick_button2 = (jc->buttons2 & (1 << 10)) ? 1 : 0;
	jc->btns.home = (jc->buttons2 & (1 << 12)) ? 1 : 0;
	
	XINPUT_GAMEPAD_DPAD_UP 	0x0001
	XINPUT_GAMEPAD_DPAD_DOWN 	0x0002
	XINPUT_GAMEPAD_DPAD_LEFT 	0x0004
	XINPUT_GAMEPAD_DPAD_RIGHT 	0x0008
	XINPUT_GAMEPAD_START 	0x0010
	XINPUT_GAMEPAD_BACK 	0x0020
	XINPUT_GAMEPAD_LEFT_THUMB 	0x0040
	XINPUT_GAMEPAD_RIGHT_THUMB 	0x0080
	XINPUT_GAMEPAD_LEFT_SHOULDER 	0x0100
	XINPUT_GAMEPAD_RIGHT_SHOULDER 	0x0200
	XINPUT_GAMEPAD_A 	0x1000
	XINPUT_GAMEPAD_B 	0x2000
	XINPUT_GAMEPAD_X 	0x4000
	XINPUT_GAMEPAD_Y	0x8000*/

	WORD remappedbtns = 0; 
	if (!settings.combineJoyCons) {
		if (jc->btns.left || jc->btns.a) {
			remappedbtns += XINPUT_GAMEPAD_A;
		}
		if (jc->btns.right || jc->btns.y) {
			remappedbtns += XINPUT_GAMEPAD_Y;
		}
		if (jc->btns.up || jc->btns.b) {
			remappedbtns += XINPUT_GAMEPAD_X;
		}
		if (jc->btns.down || jc->btns.x) {
			remappedbtns += XINPUT_GAMEPAD_B;
		}
	}
	else {
		if (jc->left_right == 1) {
			remappedbtnsl = 0;
			if (jc->btns.sr) { //run
				remappedbtnsl += XINPUT_GAMEPAD_LEFT_THUMB;
			}
			if (jc->btns.sl) { //Sprint
				remappedbtnsl += XINPUT_GAMEPAD_RIGHT_THUMB;
			}
			if (jc->btns.up) {
				remappedbtnsl += XINPUT_GAMEPAD_DPAD_UP;
			}
			if (jc->btns.down) { 
				remappedbtnsl += XINPUT_GAMEPAD_DPAD_DOWN;
			}
			if (jc->btns.left) {
				remappedbtnsl += XINPUT_GAMEPAD_DPAD_LEFT;
			}
			if (jc->btns.right) {
				remappedbtnsl += XINPUT_GAMEPAD_DPAD_RIGHT;
			}
			if (jc->btns.minus) { //Squat
				remappedbtnsl += XINPUT_GAMEPAD_BACK;
			}
		}
		if (jc->left_right == 2) {
			remappedbtnsr = 0;
			if (jc->btns.sr) { //Lightpull
				remappedbtnsr += XINPUT_GAMEPAD_LEFT_SHOULDER;
			}
			if (jc->btns.sl) { //Lightpress
				remappedbtnsr += XINPUT_GAMEPAD_RIGHT_SHOULDER;
			}
			if (jc->btns.a) {
				remappedbtnsr += XINPUT_GAMEPAD_X;
			}
			if (jc->btns.y) {
				remappedbtnsr += XINPUT_GAMEPAD_B;
			}
			if (jc->btns.b) {
				remappedbtnsr += XINPUT_GAMEPAD_Y;
			}
			if (jc->btns.x) {
				remappedbtnsr += XINPUT_GAMEPAD_A;
			}
			if (jc->btns.r) { //Heavypress
				RightTrigger = 255;
			}
			else {
				RightTrigger = 0;
			}
			if (jc->btns.zr) { //Heavypull
				LeftTrigger = 255;
			}
			else {
				LeftTrigger = 0;
			}
			if (jc->btns.home) {
				remappedbtnsr += XINPUT_GAMEPAD_START;
			}
			if (jc->btns.plus) {
				remappedbtnsr += XINPUT_GAMEPAD_BACK;
			}
		}
	}

	//Normalize sticks
	if (sThumbLX >= MaxStick) {
		sThumbLX = MaxStick;
	}
	if (sThumbLX <= -MaxStick) {
		sThumbLX = -MaxStick;
	}
	if (sThumbLY >= MaxStick) {
		sThumbLY = MaxStick;
	}
	if (sThumbLY <= -MaxStick) {
		sThumbLY = -MaxStick;
	}

	if (settings.reverseX) {
		sThumbLX = -sThumbLX;
		sThumbRX = -sThumbRX;
	}
	if (settings.reverseY) {
		sThumbLY = -sThumbLY;
		sThumbRY = -sThumbRY;
	}

	//Work out what the report should be
	if (settings.combineJoyCons) {
		report.wButtons = remappedbtnsr + remappedbtnsl;
		report.bLeftTrigger = LeftTrigger;
		report.bRightTrigger = RightTrigger;
		report.sThumbLX = sThumbLX;
		report.sThumbLY = sThumbLY;
		report.sThumbRX = sThumbRX;
		report.sThumbRY = sThumbRY;
	}
	else {
		report.wButtons = remappedbtns;
		//report.sThumbLX = sThumbLY;
		//report.sThumbLY = -sThumbLX;
		if (settings.RingconToAnalog || settings.RingconFullRH) {
			if (settings.RingconToLeftAnalog)
			{
				report.sThumbLX = sThumbLX;
				report.sThumbLY = sThumbLY;
			}
			else
			{
				report.sThumbRX = sThumbRX;
				report.sThumbRY = sThumbRY;
			}

			//report.sThumbLX = sThumbLX;
			//report.sThumbLY = sThumbLY;
			//report.sThumbRX = sThumbLX;
			//report.sThumbRY = sThumbLY;
			//printf("%i  %i  %i  %i\n", sThumbLY, sThumbRY, report.sThumbLY, report.sThumbRY);
		}
	}

	//Send data to Vigem
	if (jc->VigemNumber == 1) {
		if (settings.combineJoyCons) {
			if (p1ready) {
				vigem_target_x360_update(client, pad1, report);
				p1ready = !p1ready;
			}
			else {
				p1ready = !p1ready;
			}
		}
		else {
			vigem_target_x360_update(client, pad1, report);
		}//Client is universal for all pads. Pad is different depending on whether this is p1 controller, p2 controller etc.
	}
	if (jc->VigemNumber == 2) {
		if (settings.combineJoyCons) {
			if (p2ready) {
				vigem_target_x360_update(client, pad2, report);
				p2ready = !p2ready;
			}
			else {
				p2ready = !p2ready;
			}
		}
		else {
			vigem_target_x360_update(client, pad2, report);
		}//Client is universal for all pads. Pad is different depending on whether this is p1 controller, p2 controller etc.
	}
}





void parseSettings2() {

	//setupConsole("Debug");

	std::map<std::string, std::string> cfg = LoadConfig("config.txt");

	settings.combineJoyCons = false;
	settings.enableGyro = false;

	settings.gyroSensitivityX = 300;
	settings.gyroSensitivityY = 300;

	settings.squatSlowsMouse = false;
	settings.marioTheme = false;

	settings.reverseX = false;
	settings.reverseY = false;

	settings.preferLeftJoyCon = false;
	settings.quickToggleGyro = false;
	settings.invertQuickToggle = false;

	//settings.dolphinPointerMode = (bool)stoi(cfg["dolphinPointerMode"]);

	settings.gyroscopeComboCode = -1;

	settings.Runpressesbutton = false;
	//settings.RingconFix = stof(cfg["ringconfix"]);
	settings.RingconFix = 0;

	settings.rununlocksgyro = false;

	settings.RingconFullRH = false;
	settings.host = "https://twitchplaysnintendoswitch.com:8110";
	settings.RingconToAnalog = true;
	//模拟左手柄，配置时为false，配置完为true
	settings.RingconToLeftAnalog = true;
	//环中间参考值
	//settings.RingconReferenceValue = stof(cfg["ringconreferencevalue"]);
	settings.RingconReferenceValue = 10.0f;
	//环数值变化倍数
	//settings.ringconMul = stof(cfg["ringconmul"]);
	settings.ringconPullMul = stof(cfg["ringconPullMul"]);
	settings.ringconPushMul = stof(cfg["ringconPushMul"]);

	settings.autoStart = false;

	#pragma region 读取配置文件


	//settings.combineJoyCons = (bool)stoi(cfg["combineJoyCons"]);
	//settings.enableGyro = (bool)stoi(cfg["gyroControls"]);

	//settings.gyroSensitivityX = stof(cfg["gyroSensitivityX"]);
	//settings.gyroSensitivityY = stof(cfg["gyroSensitivityY"]);

	//settings.squatSlowsMouse = (bool)stoi(cfg["squatSlowsMouse"]);
	//settings.marioTheme = (bool)stoi(cfg["marioTheme"]);

	//settings.reverseX = (bool)stoi(cfg["reverseX"]);
	//settings.reverseY = (bool)stoi(cfg["reverseY"]);

	//settings.preferLeftJoyCon = (bool)stoi(cfg["preferLeftJoyCon"]);
	//settings.quickToggleGyro = (bool)stoi(cfg["quickToggleGyro"]);
	//settings.invertQuickToggle = (bool)stoi(cfg["invertQuickToggle"]);

	////settings.dolphinPointerMode = (bool)stoi(cfg["dolphinPointerMode"]);

	//settings.gyroscopeComboCode = stoi(cfg["gyroscopeComboCode"]);

	//settings.Runpressesbutton = (bool)stoi(cfg["runPressesButton"]);
	//settings.RingconFix = stof(cfg["ringconfix"]);

	//settings.rununlocksgyro = (bool)stoi(cfg["rununlocksgyro"]);

	//settings.RingconFullRH = (bool)stoi(cfg["ringconfullrh"]);
	//settings.host = cfg["host"];
	//settings.RingconToAnalog = (bool)stoi(cfg["ringcontoanalog"]);
	//settings.RingconToLeftAnalog = (bool)stoi(cfg["ringcontoleftanalog"]);
	//settings.RingconReferenceValue = stof(cfg["ringconreferencevalue"]);
	//settings.ringconMul = stof(cfg["ringconmul"]);

	//settings.autoStart = (bool)stoi(cfg["autoStart"]);
	#pragma endregion

}

void start();

void pollLoop() {

	// poll joycons:
	for (int i = 0; i < joycons.size(); ++i) {

		Joycon* jc = &joycons[i];

		// choose a random joycon to reduce bias / figure out the problem w/input lag:
		//Joycon *jc = &joycons[rand_range(0, joycons.size()-1)];

		if (!jc->handle) { continue; }

		hid_set_nonblocking(jc->handle, 1);

		// get input:
		memset(buf, 0, 65);

		// get current time
		std::chrono::steady_clock::time_point tNow = std::chrono::high_resolution_clock::now();

		auto timeSincePoll = std::chrono::duration_cast<std::chrono::microseconds>(tNow - tracker.tPolls[i]);

		// time spent sleeping (0):
		double timeSincePollMS = timeSincePoll.count() / 1000.0;

		if (timeSincePollMS > (1000.0 / settings.pollsPerSec)) {
			jc->send_command(0x1E, buf, 0);
			tracker.tPolls[i] = std::chrono::high_resolution_clock::now();
		}


		//hid_read(jc->handle, buf, 0x40);
		hid_read_timeout(jc->handle, buf, 0x40, 20);

		handle_input(jc, buf, 0x40);
	}

	// update vigem:
	for (int i = 0; i < joycons.size(); ++i) {
		updateVigEmDevice2(&joycons[i]);
	}

	accurateSleep(settings.timeToSleepMS);


	if (settings.restart) {
		settings.restart = false;
		//start();
	}
}



void start() {
	// set infinite reconnect attempts
	myClient.set_reconnect_attempts(999999999999);

	int read;	// number of bytes read
	int written;// number of bytes written
	const char* device_name;

	// Enumerate and print the HID devices on the system
	struct hid_device_info* devs, * cur_dev;

	res = hid_init();

	// hack:
	for (int i = 0; i < 100; ++i) {
		tracker.tPolls.push_back(std::chrono::high_resolution_clock::now());
	}


init_start:

	devs = hid_enumerate(JOYCON_VENDOR, 0x0);
	cur_dev = devs;
	while (cur_dev) {

		// identify by vendor:
		if (cur_dev->vendor_id == JOYCON_VENDOR) {

			// bluetooth, left / right joycon:
			if (cur_dev->product_id == JOYCON_L_BT || cur_dev->product_id == JOYCON_R_BT) {
				Joycon jc = Joycon(cur_dev);
				joycons.push_back(jc);
			}

			// pro controller:
			if (cur_dev->product_id == PRO_CONTROLLER) {
				Joycon jc = Joycon(cur_dev);
				joycons.push_back(jc);
			}

		}


		cur_dev = cur_dev->next;
	}
	hid_free_enumeration(devs);



	// init joycons:
	if (settings.usingGrip) {
		for (int i = 0; i < joycons.size(); ++i) {
			joycons[i].init_usb();
		}
	}
	else {
		for (int i = 0; i < joycons.size(); ++i) {
			joycons[i].init_bt();
		}
	}

	if (settings.combineJoyCons) {
		int counter = 0;
		for (int i = 0; i < joycons.size(); ++i) {
			joycons[i].VigemNumber = (counter / 2) + 1;
			joycons[i].deviceNumber = (counter % 2 ? 1 : 0);
			counter++;
		}
	}
	else {
		for (int i = 0; i < joycons.size(); ++i) {
			joycons[i].VigemNumber = i + 1;
			joycons[i].deviceNumber = 0;// left
		}
	}

	for (int i = 0; i < joycons.size(); ++i) {
		if (joycons[i].VigemNumber == 1 && joycons[i].deviceNumber == 0) {
			pad1 = vigem_target_x360_alloc();
			const auto pir1 = vigem_target_add(client, pad1);
			XUSB_REPORT_INIT(&report);
		}
		if (joycons[i].VigemNumber == 2 && joycons[i].deviceNumber == 0) {
			pad2 = vigem_target_x360_alloc();
			const auto pir2 = vigem_target_add(client, pad2);
		}
	}

	// initial poll:
	pollLoop();

	// set lights:
	printf("setting LEDs...\n");
	for (int r = 0; r < 5; ++r) {
		for (int i = 0; i < joycons.size(); ++i) {
			Joycon* jc = &joycons[i];
			// Player LED Enable
			memset(buf, 0x00, 0x40);
			if (i == 0) {
				buf[0] = 0x0 | 0x0 | 0x0 | 0x1;		// solid 1
			}
			if (i == 1) {
				if (settings.combineJoyCons) {
					buf[0] = 0x0 | 0x0 | 0x0 | 0x1; // solid 1
				}
				else if (!settings.combineJoyCons) {
					buf[0] = 0x0 | 0x0 | 0x2 | 0x0; // solid 2
				}
			}
			//buf[0] = 0x80 | 0x40 | 0x2 | 0x1; // Flash top two, solid bottom two
			//buf[0] = 0x8 | 0x4 | 0x2 | 0x1; // All solid
			//buf[0] = 0x80 | 0x40 | 0x20 | 0x10; // All flashing
			//buf[0] = 0x80 | 0x00 | 0x20 | 0x10; // All flashing except 3rd light (off)

			jc->send_subcommand(0x01, 0x30, buf, 1);

		}
	}

	//Set Ringcon IMU state
	for (int i = 0; i < joycons.size(); ++i) {
		Joycon* jc = &joycons[i];
		if (jc->left_right == 2 && jc->ringconattached) {
			ringconattached = true;
		}
	}

	// give a small rumble to all joycons:
	printf("vibrating JoyCon(s).\n");
	for (int k = 0; k < 1; ++k) {
		for (int i = 0; i < joycons.size(); ++i) {
			joycons[i].rumble(100, 1);
			Sleep(20);
			joycons[i].rumble(10, 3);
		}
	}

	// Mfosses clever Mario theme
	// Plays the Mario theme on the JoyCons:
	// I'm bad with music I just did this by
	// using a video of a piano version of the mario theme.
	// maybe eventually I'll be able to play something like sound files.

	// notes arbitrarily defined:
#define C3 110
#define D3 120
#define E3 130
#define F3 140
#define G3 150
#define G3A4 155
#define A4 160
#define A4B4 165
#define B4 170
#define C4 180
#define D4 190
#define D4E4 195
#define E4 200
#define F4 210
#define F4G4 215
#define G4 220
#define A5 230
#define B5 240
#define C5 250



	if (settings.marioTheme) {
		for (int i = 0; i < 1; ++i) {

			printf("Playing mario theme...\n");

			float spd = 1;
			float spd2 = 1;

			//goto N1;

			Joycon joycon = joycons[0];

			Sleep(1000);

			joycon.rumble(mk_odd(E4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// E2
			Sleep(100 / spd2);
			joycon.rumble(mk_odd(E4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// E2
			Sleep(100 / spd2);
			joycon.rumble(mk_odd(E4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// E2
			Sleep(100 / spd2);
			joycon.rumble(mk_odd(C4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// C2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(E4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// E2
			Sleep(100 / spd2);
			joycon.rumble(mk_odd(G4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// G2
			Sleep(400 / spd2);

			joycon.rumble(mk_odd(A4), 1); Sleep(400 / spd); joycon.rumble(1, 3);	// too low for joycon
			Sleep(50 / spd2);

			joycon.rumble(mk_odd(C4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// C2
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(G3), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// G1
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(E3), 2); Sleep(200 / spd); joycon.rumble(1, 3);	// E1
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(A4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// A2

			Sleep(100 / spd2);
			joycon.rumble(mk_odd(B4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// B2

			Sleep(50 / spd2);
			joycon.rumble(mk_odd(A4B4), 1); Sleep(200 / spd); joycon.rumble(1, 3);// A2-B2?
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(A4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// A2
			Sleep(100 / spd2);
			joycon.rumble(mk_odd(G3), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// G1


			Sleep(100 / spd2);
			joycon.rumble(mk_odd(E4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// E2
			Sleep(100 / spd2);
			joycon.rumble(mk_odd(G4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// G2
			Sleep(100 / spd2);
			joycon.rumble(mk_odd(A5), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// A3



			Sleep(200 / spd2);
			joycon.rumble(mk_odd(F4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// F2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(G4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// G2

			Sleep(200 / spd2);
			joycon.rumble(mk_odd(E4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// E2

			Sleep(50 / spd2);
			joycon.rumble(mk_odd(C4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// C2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(D4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// D2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(B4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// B2


			Sleep(200 / spd2);
			joycon.rumble(mk_odd(C4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// C2
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(G3), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// G1
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(E3), 2); Sleep(200 / spd); joycon.rumble(1, 3);	// E1

			Sleep(200 / spd2);
			joycon.rumble(mk_odd(A4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// A2
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(B4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// B2
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(A4B4), 1); Sleep(200 / spd); joycon.rumble(1, 3);// A2-B2?
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(A4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// A2
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(G4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// G2


			Sleep(100 / spd2);
			joycon.rumble(mk_odd(E4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// E2
			Sleep(100 / spd2);
			joycon.rumble(mk_odd(G4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// G2
			Sleep(100 / spd2);
			joycon.rumble(mk_odd(A5), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// A3
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(F4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// F2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(G4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// G2
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(E4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// E2


			Sleep(200 / spd2);
			joycon.rumble(mk_odd(C4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// C2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(D4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// D2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(B4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// B2

																					// new:

			Sleep(500 / spd2);

			joycon.rumble(mk_odd(G4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// G2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(F4G4), 1); Sleep(200 / spd); joycon.rumble(1, 3);// F2-G2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(F4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// F2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(D4E4), 1); Sleep(200 / spd); joycon.rumble(1, 3);// D2-E2
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(E4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// E2

			Sleep(200 / spd2);

			joycon.rumble(mk_odd(G3A4), 1); Sleep(200 / spd); joycon.rumble(1, 3);// G1-A2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(A4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// A2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(C4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// C2


			Sleep(200 / spd2);
			joycon.rumble(mk_odd(A4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// A2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(C4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// C2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(D4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// D2


			Sleep(300 / spd2);

			joycon.rumble(mk_odd(G4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// G2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(F4G4), 1); Sleep(200 / spd); joycon.rumble(1, 3);// F2-G2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(F4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// F2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(D4E4), 1); Sleep(200 / spd); joycon.rumble(1, 3);// D2-E2
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(E4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// E2


																					// three notes:
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(C5), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// C3
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(C3), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// C3
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(C3), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// C3


		N1:


			Sleep(500 / spd2);
			joycon.rumble(mk_odd(G4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// G2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(F4G4), 1); Sleep(200 / spd); joycon.rumble(1, 3);// F2G2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(F4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// F2

			Sleep(50 / spd2);
			joycon.rumble(mk_odd(D4E4), 1); Sleep(200 / spd); joycon.rumble(1, 3);// D2E2

			Sleep(200 / spd2);
			joycon.rumble(mk_odd(E4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// E2



			Sleep(200 / spd2);
			joycon.rumble(mk_odd(G3A4), 1); Sleep(200 / spd); joycon.rumble(1, 3);// G1A2

			Sleep(50 / spd2);
			joycon.rumble(mk_odd(A4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// A2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(C4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// C2
			Sleep(100 / spd2);
			joycon.rumble(mk_odd(A4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// A2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(C4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// C2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(D4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// D2


			Sleep(300 / spd2);
			joycon.rumble(mk_odd(D4E4), 1); Sleep(200 / spd); joycon.rumble(1, 3);// D2E2
			Sleep(300 / spd2);
			joycon.rumble(mk_odd(D4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// D2
			Sleep(300 / spd2);
			joycon.rumble(mk_odd(C4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// C2


			Sleep(800 / spd2);


			joycon.rumble(mk_odd(G4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// G2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(F4G4), 1); Sleep(200 / spd); joycon.rumble(1, 3);// F2G2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(F4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// F2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(D4E4), 1); Sleep(200 / spd); joycon.rumble(1, 3);// D2E2
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(E4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// E2

			Sleep(200 / spd2);


			joycon.rumble(mk_odd(G3A4), 1); Sleep(200 / spd); joycon.rumble(1, 3);// G1A2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(A4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// A2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(C4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// C2

			Sleep(200 / spd2);

			joycon.rumble(mk_odd(A4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// A2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(C4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// C2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(D4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// D2

			Sleep(300 / spd2);


			joycon.rumble(mk_odd(G4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// G2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(F4G4), 1); Sleep(200 / spd); joycon.rumble(1, 3);// F2G2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(F4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// F2


			Sleep(50 / spd2);
			joycon.rumble(mk_odd(D4E4), 1); Sleep(200 / spd); joycon.rumble(1, 3);// D2E2
			Sleep(100 / spd2);
			joycon.rumble(mk_odd(E4), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// E2

																					// 30 second mark

																					// three notes:

			Sleep(300 / spd2);
			joycon.rumble(mk_odd(C5), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// C3
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(C5), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// C3
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(C5), 1); Sleep(200 / spd); joycon.rumble(1, 3);	// C3


			Sleep(1000);
		}
	}


#define MusicOffset 600

	// notes in hertz:
#define C3 131 + MusicOffset
#define D3 146 + MusicOffset
#define E3 165 + MusicOffset
#define F3 175 + MusicOffset
#define G3 196 + MusicOffset
#define G3A4 208 + MusicOffset
#define A4 440 + MusicOffset
#define A4B4 466 + MusicOffset
#define B4 494 + MusicOffset
#define C4 262 + MusicOffset
#define D4 294 + MusicOffset
#define D4E4 311 + MusicOffset
#define E4 329 + MusicOffset
#define F4 349 + MusicOffset
#define F4G4 215 + MusicOffset
#define G4 392 + MusicOffset
#define A5 880 + MusicOffset
#define B5 988 + MusicOffset
#define C5 523 + MusicOffset

#define hfa 0xb0	// 8a
#define lfa 0x006c	// 8062


	if (false) {
		for (int i = 0; i < 1; ++i) {

			printf("Playing mario theme...\n");

			float spd = 1;
			float spd2 = 1;

			Joycon joycon = joycons[0];

			Sleep(1000);

			joycon.rumble3(E4, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// E2
			Sleep(100 / spd2);
			joycon.rumble3(E4, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// E2
			Sleep(100 / spd2);
			joycon.rumble3(E4, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// E2
			Sleep(100 / spd2);
			joycon.rumble3(C4, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// C2
			Sleep(50 / spd2);
			joycon.rumble3(E4, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// E2
			Sleep(100 / spd2);
			joycon.rumble3(G4, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// G2
			Sleep(400 / spd2);

			joycon.rumble3(A4, hfa, lfa); Sleep(400 / spd); joycon.rumble(1, 3);	// too low for joycon
			Sleep(50 / spd2);

			joycon.rumble3(C4, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// C2
			Sleep(200 / spd2);
			joycon.rumble3(G3, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// G1
			Sleep(200 / spd2);
			joycon.rumble3(E3, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// E1
			Sleep(200 / spd2);
			joycon.rumble3(A4, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// A2

			Sleep(100 / spd2);
			joycon.rumble3(B4, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// B2

			Sleep(50 / spd2);
			joycon.rumble3(A4B4, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);// A2-B2?
			Sleep(50 / spd2);
			joycon.rumble3(A4, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// A2
			Sleep(100 / spd2);
			joycon.rumble3(G3, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// G1


			Sleep(100 / spd2);
			joycon.rumble3(E4, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// E2
			Sleep(100 / spd2);
			joycon.rumble3(G4, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// G2
			Sleep(100 / spd2);
			joycon.rumble3(A5, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// A3



			Sleep(200 / spd2);
			joycon.rumble3(F4, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// F2
			Sleep(50 / spd2);
			joycon.rumble3(G4, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// G2

			Sleep(200 / spd2);
			joycon.rumble3(E4, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// E2

			Sleep(50 / spd2);
			joycon.rumble3(C4, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// C2
			Sleep(50 / spd2);
			joycon.rumble3(D4, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// D2
			Sleep(50 / spd2);
			joycon.rumble3(B4, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// B2


			Sleep(200 / spd2);
			joycon.rumble3(C4, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// C2
			Sleep(200 / spd2);
			joycon.rumble3(G3, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// G1
			Sleep(200 / spd2);
			joycon.rumble3(E3, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// E1

			Sleep(200 / spd2);
			joycon.rumble3(A4, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// A2
			Sleep(200 / spd2);
			joycon.rumble3(B4, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// B2
			Sleep(200 / spd2);
			joycon.rumble3(A4B4, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);// A2-B2?
			Sleep(50 / spd2);
			joycon.rumble3(A4, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// A2
			Sleep(200 / spd2);
			joycon.rumble3(G4, hfa, lfa); Sleep(200 / spd); joycon.rumble(1, 3);	// G2


			Sleep(1000);
		}
	}

	printf("Done.\n");

	if (settings.RingconFullRH) {
		printf("\n CAUTION: Do not use heavy press when the Ringcon is sideways. It may damage the flex sensor.");
	};




}



void actuallyQuit() {


	vigem_target_remove(client, pad1);
	vigem_target_free(pad1);
	vigem_target_remove(client, pad2);
	vigem_target_free(pad2);
	vigem_disconnect(client);
	vigem_free(client);
	/*
	for (int i = 0; i < joycons.size(); ++i) {
		buf[0] = 0x0; // disconnect
		joycons[i].send_subcommand(0x01, 0x06, buf, 1);
	}*/

	if (settings.usingGrip) {
		for (int i = 0; i < joycons.size(); ++i) {
			joycons[i].deinit_usb();
		}
	}
	// Finalize the hidapi library
	res = hid_exit();
}


#pragma region 原窗口
// ----------------------------------------------------------------------------
// constants
// ----------------------------------------------------------------------------

// control ids
enum {
	SpinTimer = wxID_HIGHEST + 1
};

// ----------------------------------------------------------------------------
// MyApp: the application object
// ----------------------------------------------------------------------------

//IMPLEMENT_APP(app);
wxIMPLEMENT_APP_NO_MAIN(MyApp);

bool MyApp::OnInit() {
	if (!wxApp::OnInit()) {
		return false;
	}

	Connect(wxID_ANY, wxEVT_IDLE, wxIdleEventHandler(MyApp::onIdle));
	auto mainFrame = new MainFrame();

	if (settings.autoStart) {
		wxCommandEvent a;
		mainFrame->onStart(a);
	}

	//new MyFrame();
	//m_myTimer.Start(0);
	return true;
}

int MyApp::OnExit() {
	//delete m_glContext;
	//delete m_glStereoContext;

	actuallyQuit();

	return wxApp::OnExit();
}

void MyApp::onIdle(wxIdleEvent& evt) {
	pollLoop();
}

/*TestGLContext& MyApp::GetContext(wxGLCanvas* canvas, bool useStereo) {
	TestGLContext* glContext;
	if (useStereo) {
		if (!m_glStereoContext) {
			// Create the OpenGL context for the first stereo window which needs it:
			// subsequently created windows will all share the same context.
			m_glStereoContext = new TestGLContext(canvas);
		}
		glContext = m_glStereoContext;
	}
	else {
		if (!m_glContext) {
			// Create the OpenGL context for the first mono window which needs it:
			// subsequently created windows will all share the same context.
			m_glContext = new TestGLContext(canvas);
		}
		glContext = m_glContext;
	}

	glContext->SetCurrent(*canvas);

	return *glContext;
}*/

void autoStart()
{
	setupConsole("Debug");

	//start();

	while (true) {
		pollLoop();
		wxYield();// so that the main window doesn't freeze
	}
}

MainFrame::MainFrame() : wxFrame(NULL, wxID_ANY, wxT("ZRingcon Driver by zhengwz(based of Ringcon Driver by RingRunnerMG)")) {

	wxPanel* panel = new wxPanel(this, wxID_ANY);

	//this->Connect(wxEVT_CLOSE_WINDOW, wxCloseEventHandler(MainFrame::onQuit), NULL, this);
	//Connect(this->GetId(), wxEVT_CLOSE_WINDOW, wxCloseEventHandler(wxCloseEventFunction, MainFrame::onQuit));
	//this->Bind(wxEVT_CLOSE_WINDOW, &MainFrame::onQuit, this);
	Connect(wxEVT_CLOSE_WINDOW, wxCloseEventHandler(MainFrame::onQuit2));

	CB1 = new wxCheckBox(panel, wxID_ANY, wxT("Combine JoyCons"), FromDIP(wxPoint(20, 20)));
	CB1->Bind(wxEVT_COMMAND_CHECKBOX_CLICKED, &MainFrame::toggleCombine, this);
	CB1->SetValue(settings.combineJoyCons);

	CB6 = new wxCheckBox(panel, wxID_ANY, wxT("Reverse Stick X"), FromDIP(wxPoint(20, 40)));
	CB6->Bind(wxEVT_COMMAND_CHECKBOX_CLICKED, &MainFrame::toggleReverseX, this);
	CB6->SetValue(settings.reverseX);
	CB7 = new wxCheckBox(panel, wxID_ANY, wxT("Reverse Stick Y"), FromDIP(wxPoint(190, 40)));
	CB7->Bind(wxEVT_COMMAND_CHECKBOX_CLICKED, &MainFrame::toggleReverseY, this);
	CB7->SetValue(settings.reverseY);

	st1 = new wxStaticText(panel, wxID_ANY, wxT("RINGCON/STRAPCON OPTIONS"), FromDIP(wxPoint(20, 80)));

	CB15 = new wxCheckBox(panel, wxID_ANY, wxT("Ringcon Full RH"), FromDIP(wxPoint(20, 100)));
	CB15->Bind(wxEVT_COMMAND_CHECKBOX_CLICKED, &MainFrame::toggleRingconFullRH, this);
	CB15->SetValue(settings.RingconFullRH);

	CB4 = new wxCheckBox(panel, wxID_ANY, wxT("Squat Slows Gyro"), FromDIP(wxPoint(190, 100)));
	CB4->Bind(wxEVT_COMMAND_CHECKBOX_CLICKED, &MainFrame::toggleSquatSlowsMouse, this);
	CB4->SetValue(settings.squatSlowsMouse);

	CB11 = new wxCheckBox(panel, wxID_ANY, wxT("Run Unlocks Gyro"), FromDIP(wxPoint(20, 120)));
	CB11->Bind(wxEVT_COMMAND_CHECKBOX_CLICKED, &MainFrame::toggleRunUnlocksGyro, this);
	CB11->SetValue(settings.rununlocksgyro);

	CB9 = new wxCheckBox(panel, wxID_ANY, wxT("Run Presses Button"), FromDIP(wxPoint(190, 120)));
	CB9->Bind(wxEVT_COMMAND_CHECKBOX_CLICKED, &MainFrame::toggleRunpressesbutton, this);
	CB9->SetValue(settings.Runpressesbutton);

	CB16 = new wxCheckBox(panel, wxID_ANY, wxT("Ringcon模拟摇杆"), FromDIP(wxPoint(20, 140)));
	CB16->Bind(wxEVT_COMMAND_CHECKBOX_CLICKED, &MainFrame::toggleRingconToAnalog, this);
	CB16->SetValue(settings.RingconToAnalog);

	CB16 = new wxCheckBox(panel, wxID_ANY, wxT("模拟左摇杆(不勾选则为右摇杆)"), FromDIP(wxPoint(190, 140)));
	CB16->Bind(wxEVT_COMMAND_CHECKBOX_CLICKED, &MainFrame::toggleRingconToLeftAnalog, this);
	CB16->SetValue(settings.RingconToLeftAnalog);

	//CB10 = new wxCheckBox(panel, wxID_ANY, wxT("Ringcon Fix"), FromDIP(wxPoint(20, 160)));
	//CB10->Bind(wxEVT_COMMAND_CHECKBOX_CLICKED, &MainFrame::toggleRingconFix, this);
	//CB10->SetValue(settings.RingconFix);
	slider3Text = new wxStaticText(panel, wxID_ANY, wxT("Ringcon Fix - Aim for ~10 (9 and 11 OK)"), FromDIP(wxPoint(20, 160)));
	slider3 = new wxSlider(panel, wxID_ANY, settings.RingconFix, -20, 20, FromDIP(wxPoint(180, 160)), FromDIP(wxSize(150, 20)), wxSL_LABELS);
	slider3->Bind(wxEVT_SLIDER, &MainFrame::setRingconFix, this);

	st1 = new wxStaticText(panel, wxID_ANY, wxT("JOYCON OPTIONS"), FromDIP(wxPoint(20, 200)));

	gyroCheckBox = new wxCheckBox(panel, wxID_ANY, wxT("Gyro Controls Mouse"), FromDIP(wxPoint(20, 220)));
	gyroCheckBox->Bind(wxEVT_COMMAND_CHECKBOX_CLICKED, &MainFrame::toggleGyro, this);
	gyroCheckBox->SetValue(settings.enableGyro);

	CB12 = new wxCheckBox(panel, wxID_ANY, wxT("Quick Toggle Gyro Controls"), FromDIP(wxPoint(190, 220)));
	CB12->Bind(wxEVT_COMMAND_CHECKBOX_CLICKED, &MainFrame::toggleQuickToggleGyro, this);
	CB12->SetValue(settings.quickToggleGyro);

	CB13 = new wxCheckBox(panel, wxID_ANY, wxT("Invert Quick Toggle"), FromDIP(wxPoint(20, 240)));
	CB13->Bind(wxEVT_COMMAND_CHECKBOX_CLICKED, &MainFrame::toggleInvertQuickToggle, this);
	CB13->SetValue(settings.invertQuickToggle);

	CB5 = new wxCheckBox(panel, wxID_ANY, wxT("Mario Theme"), FromDIP(wxPoint(190, 240)));
	CB5->Bind(wxEVT_COMMAND_CHECKBOX_CLICKED, &MainFrame::toggleMario, this);
	CB5->SetValue(settings.marioTheme);

	CB8 = new wxCheckBox(panel, wxID_ANY, wxT("Prefer Left JoyCon for Gyro Controls"), FromDIP(wxPoint(20, 260)));
	CB8->Bind(wxEVT_COMMAND_CHECKBOX_CLICKED, &MainFrame::togglePreferLeftJoyCon, this);
	CB8->SetValue(settings.preferLeftJoyCon);

	//CB14 = new wxCheckBox(panel, wxID_ANY, wxT("Dolphin Mode"), FromDIP(wxPoint(190, 120)));
	//CB14->Bind(wxEVT_COMMAND_CHECKBOX_CLICKED, &MainFrame::toggleDolphinPointerMode, this);
	//CB14->SetValue(settings.dolphinPointerMode);

	slider1Text = new wxStaticText(panel, wxID_ANY, wxT("Gyro Controls Sensitivity X"), FromDIP(wxPoint(20, 300)));
	slider1 = new wxSlider(panel, wxID_ANY, settings.gyroSensitivityX, -1000, 1000, FromDIP(wxPoint(180, 280)), FromDIP(wxSize(150, 20)), wxSL_LABELS);
	slider1->Bind(wxEVT_SLIDER, &MainFrame::setGyroSensitivityX, this);


	slider2Text = new wxStaticText(panel, wxID_ANY, wxT("Gyro Controls Sensitivity Y"), FromDIP(wxPoint(20, 340)));
	slider2 = new wxSlider(panel, wxID_ANY, settings.gyroSensitivityY, -1000, 1000, FromDIP(wxPoint(180, 320)), FromDIP(wxSize(150, 20)), wxSL_LABELS);
	slider2->Bind(wxEVT_SLIDER, &MainFrame::setGyroSensitivityY, this);




	gyroComboCodeText = new wxStaticText(panel, wxID_ANY, wxT("Gyro Combo Code: "), FromDIP(wxPoint(20, 320)));

	// st1 = new wxStaticText(panel, wxID_ANY, wxT("Change the default settings and more in the config file!"), FromDIP(wxPoint(20, 340)));

	//wxString version;
	//version.Printf("JoyCon-Driver version %s\n", settings.version);
	//st2 = new wxStaticText(panel, wxID_ANY, version, FromDIP(wxPoint(20, 330)));

	startButton = new wxButton(panel, wxID_EXIT, wxT("启动"), FromDIP(wxPoint(150, 380)));
	startButton->Bind(wxEVT_BUTTON, &MainFrame::onStart, this);

	quitButton = new wxButton(panel, wxID_EXIT, wxT("退出"), FromDIP(wxPoint(250, 380)));
	quitButton->Bind(wxEVT_BUTTON, &MainFrame::onQuit, this);

	//updateButton = new wxButton(panel, wxID_EXIT, wxT("Check for update"), FromDIP(wxPoint(18, 360)));
	//updateButton->Bind(wxEVT_BUTTON, &MainFrame::onUpdate, this);

	donateButton = new wxButton(panel, wxID_EXIT, wxT("配置说明"), FromDIP(wxPoint(190, 5)));
	donateButton->Bind(wxEVT_BUTTON, &MainFrame::onDetail, this);

	donateButton = new wxButton(panel, wxID_EXIT, wxT("赞助"), FromDIP(wxPoint(50, 380)));
	donateButton->Bind(wxEVT_BUTTON, &MainFrame::onDonate, this);

	(SetClientSize(FromDIP(360), FromDIP(420)));
	//Show();//autoStart时不显示窗口

	autoStart();
	//checkForUpdate();
}


void MainFrame::onStart(wxCommandEvent&) {
	setupConsole("Debug");

	start();

	while (true) {
		pollLoop();
		wxYield();// so that the main window doesn't freeze
	}
}

void MainFrame::onQuit(wxCommandEvent&) {
	actuallyQuit();
	exit(0);
}

void MainFrame::onQuit2(wxCloseEvent&) {
	actuallyQuit();
	exit(0);
}

void MainFrame::onDetail(wxCommandEvent&) {
	wxString alert;
	alert.Printf("本说明仅针对健身环游戏\n\n首次使用时需先进行配置\n1.仅勾选Ringcon模拟摇杆，不勾选模拟左摇杆\n2.先点启动并成功连接手柄后再运行yuzu模拟器\n3.在yuzu控制设置的高级项设置Ring Controller\n4.选中Push后压一下环，即可映射上轴1\n5.右键Push框，点Invert asix交换Push和Pull的响应\n6.保存后即完成基础配置\n7.首次配置完成需重启本软件及yuzu!!!!!\n\n后续使用仅需打开本程序后直接点击启动即可\n(同时勾选模拟摇杆及模拟左摇杆)\n1.玩家1设置为右Joycon手柄，输入设备设置为Jon-con Right\n2.设置玩家1的体感，摇晃右手柄设置上sdl\n3.玩家2设置为双Joycon手柄，输入设备设置为Dual Joy-con\n4.分别设置玩家2的左右体感为对应的手柄\n5.保存即可开玩\n\n如果您觉得本软件对您有帮助，可进行友情赞助\n赞助充足有利于后续研发\n\n本软件仅供学习，请勿用于商业用途");
	wxMessageBox(alert);
}

void MainFrame::onDonate(wxCommandEvent&) {
	wxString alert;
	alert.Printf("Thank you very much!\n\n您可以通过PayPal友情赞助:\npaypal：zhengwz999@gmail.com\n\n本次魔改仅针对健身环游戏\n\n也可用于模拟手柄游玩其他手柄游戏(如果你想挑战自己)\n其余游戏可自行摸索\n\n如果您觉得本软件对您有帮助，可进行友情赞助\n赞助充足有利于后续研发\n\n本软件仅供学习，请勿用于商业用途");
	wxMessageBox(alert);
}

void MainFrame::toggleCombine(wxCommandEvent&) {
	settings.combineJoyCons = !settings.combineJoyCons;
}

void MainFrame::toggleGyro(wxCommandEvent&) {
	settings.enableGyro = !settings.enableGyro;
}

void MainFrame::toggleSquatSlowsMouse(wxCommandEvent&) {
	settings.squatSlowsMouse = !settings.squatSlowsMouse;
}

void MainFrame::toggleMario(wxCommandEvent&) {
	settings.marioTheme = !settings.marioTheme;
}

void MainFrame::toggleReverseX(wxCommandEvent&) {
	settings.reverseX = !settings.reverseX;
}

void MainFrame::toggleReverseY(wxCommandEvent&) {
	settings.reverseY = !settings.reverseY;
}

void MainFrame::togglePreferLeftJoyCon(wxCommandEvent&) {
	settings.preferLeftJoyCon = !settings.preferLeftJoyCon;
}

void MainFrame::toggleQuickToggleGyro(wxCommandEvent&) {
	settings.quickToggleGyro = !settings.quickToggleGyro;
}

void MainFrame::toggleInvertQuickToggle(wxCommandEvent&) {
	settings.invertQuickToggle = !settings.invertQuickToggle;
}

void MainFrame::toggleDolphinPointerMode(wxCommandEvent&) {
	//settings.dolphinPointerMode = !settings.dolphinPointerMode;
}

void MainFrame::toggleRunpressesbutton(wxCommandEvent&) {
	settings.Runpressesbutton = !settings.Runpressesbutton;
}

void MainFrame::setRingconFix(wxCommandEvent&) {
	settings.RingconFix = slider3->GetValue();
}

void MainFrame::toggleRunUnlocksGyro(wxCommandEvent&) {
	settings.rununlocksgyro = !settings.rununlocksgyro;
}

void MainFrame::setGyroSensitivityX(wxCommandEvent&) {
	settings.gyroSensitivityX = slider1->GetValue();
}

void MainFrame::setGyroSensitivityY(wxCommandEvent&) {
	settings.gyroSensitivityY = slider2->GetValue();
}

void MainFrame::toggleRingconFullRH(wxCommandEvent&) {
	settings.RingconFullRH = !settings.RingconFullRH;
}

void MainFrame::toggleRingconToAnalog(wxCommandEvent&) {
	settings.RingconToAnalog = !settings.RingconToAnalog;
}

void MainFrame::toggleRingconToLeftAnalog(wxCommandEvent&) {
	settings.RingconToLeftAnalog = !settings.RingconToLeftAnalog;
}

void setGyroComboCodeText(int code) {
	wxString text;
	text.Printf("Gyro Combo Code: %d\n", code);
	gyroComboCodeText->SetLabel(text);
}

wxString glGetwxString(GLenum name) {
	const GLubyte* v = glGetString(name);
	if (v == 0) {
		// The error is not important. It is GL_INVALID_ENUM.
		// We just want to clear the error stack.
		glGetError();

		return wxString();
	}

	return wxString((const char*)v);
}

//int main(int argc, char *argv[]) {
int wWinMain(HINSTANCE hInstance, HINSTANCE prevInstance, LPWSTR cmdLine, int cmdShow) {
	parseSettings2();
	wxEntry(hInstance);
	return 0;
}

#pragma endregion

bool stop = false;
//新增，1.04版DLL还未有此接口，备用后面可能的未知原因需要重开
extern "C" _declspec(dllexport) int APIStop()
{
	stop = true;
	return 0;
}

bool isstart = false;
extern "C" _declspec(dllexport) int APIStart()
{
	if (!isstart)
	{
		isstart = true;
		stop = false;

		//setupConsole("Debug");//dos日志窗口
		parseSettings2();
		start();
		while (true) {
			pollLoop();
			if (stop)
			{
				break;
			}
		}
		isstart = false;
	}
	return 0;
}

extern "C" _declspec(dllexport) int ChangeMode(bool config)
{
	settings.RingconToLeftAnalog = !config;
	printf("Changemode");
	return 0;
}

extern "C" _declspec(dllexport) int GetStatus()
{
	if (RingconStatus == 0)
	{
		for (int i = 0; i < joycons.size(); ++i) {
			if (joycons[i].Step == 6 && joycons[i].Status == -1)
				return -1;
		}
	}
	return RingconStatus;
}

extern "C" _declspec(dllexport) double GetRingconReferenceValue()
{
	return settings.RingconReferenceValue;
}

extern "C" _declspec(dllexport) double GetRingconValue()
{
	return Ringcon_Sum - settings.RingconReferenceValue;
}


