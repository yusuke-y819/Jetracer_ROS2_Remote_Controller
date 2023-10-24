/*!
  @file
  @brief : USB ジョイスティック用のライブラリ
  全キーに対応 バージョン

  @author : Kiyoshi MATSUO
*/

#include <fcntl.h>
#include <stdio.h>
#include <sys/termios.h>
#include <unistd.h>

#include "controller.hpp"

//extern int FD;
int FD;

/*!
  @brief : ジョイスティックと接続する関数

  @retval 1: 接続失敗
  @retval 0: 接続成功
*/
extern int openJoystick(void)
{
	// 読み込み専用で、ジョイスティック用に fd を開く
	if ((FD = open(DEV_NAME, O_RDONLY, 0)) == -1){
		fprintf(stderr,"Can't open %s\n",DEV_NAME);
		return 1;
	}
	tcflush(FD,TCIOFLUSH);

	return 0;
}

/*!
  @brief : コントローラからのデータを読み込む関数
  @param : *p_time タイムスタンプ
  @param : handle ハンドルからの入力データ (8bit)
  @param : pedal ペダルからの入力データ
  @param : cross 十字キーからの入力データ
  @param : button ボタンからの入力データ

  @retval 1 : read 失敗
  @retval 0 : 成功
*/
extern int readJoystick(int *p_time, int *handle, int pedal[], char cross[], char button[])
{
	// コントローラからの入力データ
	struct js_event_data js;
	// fprintf(stderr, "read Start\n");
	read(FD, &js, JS_EVENT_RETURN);
	// fprintf(stderr, "read end\n");

	// 値の代入 js.type によって、ジョイスティック入力とボタン入力をシフトしている

	// タイムスタンプの入力 マシンタイムを使って比較したところ単位は [msec] かな
	// けれど、たまに最大 10 [msec] の誤差がある。連射しているとかの場合は
	// 問題なく 0.05 [msec] ぐらいの誤差しかない
	*p_time = js.time;

	// axesによる入力の場合
	if (js.type == 2)
		{
			switch (js.number) {

				// ハンドル入力 1013 で割る理由は入力データを 8 bit にするため
			case 0:
				*handle = js.value;
				break;
                // ペダル入力　(クラッチ、アクセル、ブレーキ)
			case 1:
				pedal[0] = -js.value + 32767;
				break;
			case 2:
				pedal[1] = -js.value + 32767;
				break;
			case 3:
				pedal[2] = -js.value + 32767;
				break;

				// 十字キーによる入力
			case 4:
				if (js.value < 0) {
					cross[0] = -1;
				}
				if (js.value > 0) {
					cross[0] = 1;
				}
				if (js.value == 0) {
					cross[0] = 0;
				}
				break;
			case 5:
				if (js.value < 0) {
					cross[1] = 1;
				}
				if (js.value > 0) {
					cross[1] = -1;
				}
				if (js.value == 0) {
					cross[1] = 0;
				}
				break;
			}
		}
	int i = 0;

	// ボタンによる入力
	if (js.type == 1) {
		for (i = 0; i < 25; ++i) {
			if (js.number == i) {
				button[i] = js.value;
			}
		}
	}
	fflush(stdout);
	// fprintf(stderr, "read end\n");
	// tcflush(FD,TCIOFLUSH);


	return 0;
}


/*!
  @brief : コントローラからデータを読み込む関数 (現在の入力状況を返す)

  @param : stick_left ジョイスティックの状態
  @param : button ボタンの状態

  @retval 1 : read 失敗
  @retval 0 : 成功
*/
extern int readNowJoystick(int stick[], char button[])
{
	struct js_real_data js;

	if (read(FD, &js, JS_REAL_RETURN) != JS_REAL_RETURN) {
		perror("\ntest: error reading");
		return 1;
	}

	stick[0] = js.x;
	stick[1] = js.y;

	int i = 0;
	int binary = 1;

	for (i = 0; i < 10 ; ++i) {
		if (js.buttons & (binary)) {
			button[i] = 1;
		}
		else {
			button[i] = 0;
		}
		binary *= 2;
	}
	return 0;
		}

/*!
  @brief : ボタン入力のイニシャライズ

  @attention : いらない
*/
void initializeButton(int *handle, int pedal[], char cross[], char button[])
{
	int i = 0;
	fprintf(stderr, "initial Start");
    *handle = 0;
	for (i = 0; i < 4; i++) {
		pedal[i] = 0;
	}
	for (i = 0; i < 4; i++) {
		cross[i] = 0;
	}
	for (i = 0; i < 25; i++) {
		button[i] = 0;
	}
}

/*!
  @brief : コントローラとの通信を終了する関数
*/
extern void closeJoystick(void) {
	tcflush(FD,TCIOFLUSH);
	close(FD);
}
