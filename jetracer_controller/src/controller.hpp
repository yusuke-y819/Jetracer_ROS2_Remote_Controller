#ifndef USBCONTROLLER_H
#define USBCONTROLLER_H

/*!
  @file
  @brief : ジョイスティック付き USB コントローラ用のライブラリ
  @author :Kiyoshi MATSUO
*/

#include <asm/types.h>
#define DEV_NAME "/dev/input/js0"


// コントローラからの入力データの型
struct js_event_data {

	// タイムスタンプ
	__u32 time;

	// 入力値
	__s16 value;

	// 入力タイプ (ボタン入力 or スティック十字キー入力 の選択)
	__u8 type;

	// ボタン番号
	__u8 number;
};

#define JS_EVENT_RETURN sizeof(struct js_event_data)

struct js_real_data {

  // ボタンの入力値 (離散的 0-512)
  int buttons;

  // 左スティックの入力値 (8bit)
  int x;

  // 右スティックの入力値 (8bit)
  int y;
};

#define JS_REAL_RETURN sizeof(struct js_real_data)

/*!
  @brief : ジョイスティックとの通信を開始する関数
  @retval 0 : 通信開始
  @retval 1 : 通信エラー
*/
extern int openJoystick(void);

/*!
  @brief : コントローラからデータを読み込む関数 (イベント発生時に更新)

  @param : *p_time タイムスタンプ
  @param : stick ジョイスティックからの入力データ (8bit)
  @param : cross 十字キーからの入力データ
  @param : button ボタンからの入力データ

  @retval 2 : read 失敗
  @retval 0 : 成功
*/
extern int readJoystick(int *p_time, int *hundle, int pedal[], char cross[], char button[]);

/*!
  @brief : コントローラからデータを読み込む関数 (現在の入力状況を返す)

  @param : stick_left ジョイスティックの状態
  @param : button ボタンの状態
*/
extern int readNowJoystick(int stick[], char button[]);

/*!
  @brief : ボタン入力用データのイニシャライズ
  @attention : いらないと思う
*/
extern void initializeButton(int *hundle, int pedal[], char cross[], char button[]);

/*!
  @brief : コントローラとの通信を終了する関数
*/
extern void closeJoystick(void);

#endif // USBCONTROLLER_H