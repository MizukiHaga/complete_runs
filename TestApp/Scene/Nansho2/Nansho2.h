#ifndef TESTAPP_NANSHO2_H_
#define TESTAPP_NANSHO2_H_

#include <kernel.h>
#include <pbio/port.h>

#include "app.h"
#include "ColorLAB.h"

#define NANSHO2_ARM_PORT PBIO_PORT_ID_C

class Nansho2Scene {
public:
  enum class RouteBranch {
    None,
    Red,
    Yellow,
    Blue,
  };

  /* ⓪ LT (スカイブルー → 黒) → 後退 → アーム上げ → 0° 補正低速直進
   *     → 色検知で停止・ブザー・少し後退・アーム下げ（分岐フラグのみ設定）
   *       ブロック色は PETRED / PETYELLOW / PETBLUE で判定
 *  ① 赤/黄/青いずれも同じ: LT(AfterColorTraceStopHeadingDeg 到達) →
 *     AfterColorStraightHeadingDeg を保って距離制御直進 → その場旋回(±90°)
 *     → ±90°直進（クルーズ→37減速→黒まで低速） → 旋回(R:左のみ+90 / L:右のみ-90) → 低速LT → LT → SKYBLUEまで → ジャイロreset
   *  ② 青(SKYBLUE)検知後（分岐フラグでルート選択。センサ値は保持しない）
 *     黄: 少し直進 → 右旋回 → 時間制御直進 → ジャイロreset → 後退（黒または青検知まで）
 *     青: 黒LT → 青LT → 少し直進 → 右旋回 → 時間制御直進 → ジャイロreset → 後退（黒または青検知まで）
 *     赤: 黒LT → 青LT → 黒LT → 青LT → 少し直進 → 右旋回 → 時間制御直進 → ジャイロreset
 *         → 後退（エンコーダ距離） → その場旋回 → 後退（黒または青検知まで）
 *         → 少し直進 → 絶対角旋回 → 低速LT → 白で停止
 *     黄/青/赤 → 共通終端: 絶対角旋回(R=+90 / L=-90、左右モータ入替) → 低速LT → 白で停止 */
  void run();

  RouteBranch routeBranch() const { return route_branch_; }

private:
  void runInitialLineTrace();
  void runShortBackAfterGray();
  void runShortBackBeforeLowerArm();
  void raiseArm();
  void lowerArm();
  void beepOnRouteBranchDetected();
  void beepOnBlackDetected();
  void beepOnBlueDetected();
  void driveForwardAndDetectRouteBranch();

  bool scanRouteBranchOnce();
  void runLineTraceUntilHeadingThenStraight();
  void runAfterColorLineTraceUntilHeading();
  void runAfterBlackLeftTurnAndBranch();
  void driveStraightUntilBlackLine(float target_heading);
  void runLineTraceUntilColor(color_detect_t color);
  void runAfterBlueByRouteBranch();
  void runYellowRouteAfterBlue();
  void runBlueRouteAfterBlue();
  void runRedRouteAfterBlue();

  float runBlackBlueForwardTurnRight(int black_blue_pairs = 1);
  float runShortForwardTurnRight();
  /* 旋回後の直進。終了条件はエンコーダ距離 [deg]（左右の平均）。
   * kp 省略時は RouteFinalApproachBalance（遅い直進）。後退なら Back 側を渡す。 */
  void runStraightAfterTurn(float heading, int left_power, int right_power,
                            int32_t target_deg);
  void runStraightAfterTurn(float heading, int left_power, int right_power,
                            int32_t target_deg, float kp, float ki, float i_max);
  /* ボトル設置後: 旋回を挟まず、後退したままラインを検知するまで下がる。
   * ignore_ms の間は判定しない（設置位置のラインを拾わないため）。
   * then_final=false なら終端旋回・LT は呼ばない。 */
  void runRouteFinalBack(float heading, int power, uint32_t ignore_ms,
                         bool then_final = true);
  void runAfterFinalStopTurnRightAndSlowTraceToWhite(bool do_turn = true);

  void turnToHeading(float target_heading, float tolerance_deg,
                     int left_power, int right_power,
                     int prefer_turn_sign = 0);
  void setupPostBlackLineTrace();

  static const char *routeBranchName(RouteBranch branch);

  RouteBranch route_branch_ = RouteBranch::None;

  /* === 共通 === */
  static constexpr uint32_t StepMs = 1U;  /* 制御ループ周期 [ms] */
  static constexpr uint32_t LtMaxMs = 8000U;  /* LT のタイムアウト [ms] */

  /* === 共通（その場旋回） === */
  static constexpr float    TurnSlowApproachDeg   = 15.0f; /* この角度以内で減速旋回 [deg] */
  static constexpr int      TurnSlowLeftPower     = 34;    /* 減速旋回 左パワー */
  static constexpr int      TurnSlowRightPower    = 34;    /* 減速旋回 右パワー */

  /* === ⓪ 開始LT（スカイブルー → 黒） === */
  static constexpr float    TracePower         = 33.0f;   /* LT 左右ベースパワー */
  static constexpr float    InitialTraceKp     = 0.45f;   /* LT PID: P */
  static constexpr float    InitialTraceKi     = 0.005f;   /* LT PID: I */
  static constexpr float    InitialTraceKd     = 0.8f;    /* LT PID: D */
  static constexpr float    InitialTraceOutMax = 100.0f;  /* LT PID 出力上限 */
  static constexpr float    InitialTraceScale  = 1.0f;    /* HCR 後も自前 PID を維持 */
  static constexpr bool     InitialTraceEdgeFollow = (ETROBO_COURSE_L == 0);
  /* InitialTraceEdgeFollow: L=右エッジ, R=左エッジ */

  /* === ⓪ 黒到達後の後退 === */
  static constexpr int      BackAfterGrayPower = 35;    /* 後退パワー */
  static constexpr uint32_t BackAfterGrayMs    = 500U;  /* 後退時間 [ms] */

  /* === ⓪ 色検知後の後退（アーム下げ前） === */
  static constexpr int      BackBeforeLowerPower = 50;    /* 後退パワー */
  static constexpr uint32_t BackBeforeLowerMs    = 300U;  /* 後退時間 [ms] */

  /* === ⓪ アーム === */
  static constexpr uint32_t ArmRaiseBeforeMs = 300U; /* 上げ前の待ち [ms] */
  static constexpr int      ArmRaisePower = 100;     /* 上げパワー */
  static constexpr int      ArmLowerPower = -100;  /* 下げパワー */
  static constexpr int32_t  ArmRaiseDeg   = 140;   /* 上げ角度 [deg] */
  /* 0 位置を過ぎて余分に下げる量 [deg]（確実に接地） */
  static constexpr int32_t  ArmLowerOvershootDeg = 25;
  static constexpr uint32_t ArmRaiseMaxMs = 4000U; /* 上げタイムアウト [ms] */
  static constexpr uint32_t ArmLowerMaxMs = 4000U; /* 下げタイムアウト [ms] */

  /* === ⓪ ブロック色検知（ヨー補正直進） === */
  static constexpr int      ColorForwardLeftPower  = 40;     /* 直進左パワー */
  static constexpr int      ColorForwardRightPower = 40;     /* 直進右パワー */
  static constexpr uint32_t ColorForwardMaxMs      = 3000U;  /* 色検知タイムアウト [ms] */
  /* 目標ヨー [deg]。L=-10 / R=+10 */
  static constexpr float    ColorForwardTargetHeadingDeg =
      ETROBO_COURSE_L ? 0.0f : 0.0f;  //ここ要検討
  /* ヨー角 PI 補正（gMotors.setBalancePid） */
  static constexpr float    ColorForwardBalanceKp         = 3.0f; /* P */
  static constexpr float    ColorForwardBalanceKi         = 0.01f; /* I */
  static constexpr float    ColorForwardBalanceIntegralMax = 50.0f;

  /* === ① 色検知後LT（AfterColorTraceStopHeadingDeg 到達で終了） === */
  static constexpr float    AfterColorTraceLeftPower  = 60.0f;  /* LT 左ベースパワー */
  static constexpr float    AfterColorTraceRightPower = 60.0f; /* LT 右ベースパワー */
  static constexpr float    AfterColorTraceKp         = 0.55f;  /* LT PID: P */
  static constexpr float    AfterColorTraceKi         = 0.025f;  /* LT PID: I */
  static constexpr float    AfterColorTraceKd         = 0.8f;   /* LT PID: D */
  static constexpr float    AfterColorTraceOutMax     = 100.0f; /* LT PID 出力上限 */
  static constexpr float    AfterColorTraceScale      = 1.0f;
  static constexpr bool     AfterColorEdgeFollow      = (ETROBO_COURSE_L != 0);
  /* AfterColorEdgeFollow: L=右エッジ, R=左エッジ */
  /* LT を終える角度の大きさ [deg]。ここだけ変えれば L/R 両方に効く */
  static constexpr float    AfterColorTraceStopHeadingMagnitudeDeg = 73.0f;
  static constexpr float    AfterColorTraceStopHeadingDeg =
      ETROBO_COURSE_L ? AfterColorTraceStopHeadingMagnitudeDeg
                      : -AfterColorTraceStopHeadingMagnitudeDeg;
  /* AfterColorTraceStopHeadingDeg: LT 終了の目標ヨー角 [deg] */
  static constexpr float    AfterColorTraceStopToleranceDeg = 2.0f;
  /* AfterColorTraceStopToleranceDeg: LT 終了の角度許容 ±[deg] */

  /* === ① LT 終了角に到達した後の直進 === */
  /* 直進中に保つ目標ヨー角 [deg]。既定は LT を終えた角度と同じ。
   * LT 終了角と別の向きで走らせたいときはここだけ変える。 */
  static constexpr float    AfterColorStraightHeadingDeg =
      AfterColorTraceStopHeadingDeg;
  static constexpr int      AfterColorStraightLeftPower  = 100;  /* 直進左パワー */
  static constexpr int      AfterColorStraightRightPower = 100;  /* 直進右パワー */
  /* 直進距離 [deg]。左右エンコーダの平均で、車輪 56mm なら 360deg ≒ 176mm */
  static constexpr int32_t  AfterColorStraightDeg        = 700;

  /* === ① その場旋回（±90°） === */
  static constexpr float    AfterPivotTurnDeg =
      ETROBO_COURSE_L ? 90.0f : -90.0f;  /* 旋回目標角 [deg] */
  static constexpr float    AfterPivotTurnToleranceDeg    = 1.64f; /* 角度許容 ±[deg] */
  static constexpr int      AfterPivotTurnLeftPower       = 0;   /* 旋回 左パワー */
  static constexpr int      AfterPivotTurnRightPower      = 36;   /* 旋回 右パワー */

  /* === ① ±90° 直進: クルーズ → 37まで減速 → 黒検知まで低速直進 === */
  static constexpr float    AfterStraightHeadingDeg = AfterPivotTurnDeg;
  /* 直進の目標ヨー角 [deg] */
  static constexpr int      AfterStraightToBlackLeftPower  = 100;    /* クルーズ左パワー */
  static constexpr int      AfterStraightToBlackRightPower = 100;    /* クルーズ右パワー */
  static constexpr int      AfterStraightSlowPower         = 50;     /* 減速後の維持パワー */
  static constexpr float    AfterStraightBalanceKp =
      ETROBO_COURSE_L ? 7.0f : 7.0f;   /* 直進補正: P（R / L） */
  static constexpr float    AfterStraightBalanceKi =
      ETROBO_COURSE_L ? 0.23f : 0.3f; /* 直進補正: I（R / L） */
  static constexpr float    AfterStraightBalanceIntegralMax =
      ETROBO_COURSE_L ? 50.0f : 50.0f; /* 直進補正: I上限（R / L） */
  /* クルーズを終える距離 [deg]。ここから減速と黒線判定を始める */
  static constexpr int32_t  AfterStraightToBlackCruiseDeg  = 1970;
  /* クルーズパワーから AfterStraightSlowPower まで落とす時間 [ms] */
  static constexpr uint32_t AfterStraightToBlackDecelMs    = 400U;
  static constexpr int32_t  AfterStraightBlackReflectionMax = 30;
  /* AfterStraightBlackReflectionMax: 黒線判定（反射光上限） */
  static constexpr uint32_t AfterStraightBlackMaxMs = 15000U;
  /* AfterStraightBlackMaxMs: 黒未検知時の安全タイムアウト [ms]（通常は黒で停止） */

  /* === ① 90°直進で黒停止後の旋回 === */
  /* R=左モータのみ(+90°) / L=右モータのみ(-90°) */
  static constexpr float    AfterBlackTurnOffsetDeg =
      ETROBO_COURSE_L ? 90.0f : -90.0f;
  static constexpr float    AfterBlackLeftTurnToleranceDeg = 1.64f; /* 角度許容 [deg] */
  static constexpr int      AfterBlackLeftTurnLeftPower  =
      ETROBO_COURSE_L ? 70 : 0;   /* R=左のみ / L=右のみ */
  static constexpr int      AfterBlackLeftTurnRightPower =
      ETROBO_COURSE_L ? 0 : 70;
  static constexpr int      AfterBlackTurnPreferSign =
      ETROBO_COURSE_L ? 1 : -1;  /* R=右旋回 / L=左旋回 */

  /* === ① 左向き旋回直後の低速LT（線に乗せる） === */
  static constexpr float    PostBlackSlowTracePower  = 37.0f; /* 低速LT パワー */
  static constexpr uint32_t PostBlackSlowTraceMs     = 300U;  /* 低速LT 時間 [ms] */
  static constexpr float    PostBlackSlowTraceKp     = 0.30f;
  static constexpr float    PostBlackSlowTraceKi     = 0.01f;
  static constexpr float    PostBlackSlowTraceKd     = 0.8f;
  static constexpr float    PostBlackSlowTraceOutMax = 100.0f;
  static constexpr float    PostBlackSlowTraceScale  = 1.0f;

  /* === ① 左向き旋回後LT（SKYBLUE①まで） === */
  static constexpr float    PostBlackTraceLeftPower  = 60.0f;  /* LT 左ベースパワー */
  static constexpr float    PostBlackTraceRightPower = 60.0f; /* LT 右ベースパワー */
  static constexpr float    PostBlackTraceKp         = 0.55;  /* LT PID: P */
  static constexpr float    PostBlackTraceKi         = 0.01f;   /* LT PID: I */
  static constexpr float    PostBlackTraceKd         = 0.8f;   /* LT PID: D */
  static constexpr float    PostBlackTraceOutMax     = 100.0f; /* LT PID 出力上限 */
  static constexpr float    PostBlackTraceScale      = 1.0f;
  static constexpr bool     PostBlackTraceEdgeFollow = (ETROBO_COURSE_L == 0);
  /* PostBlackTraceEdgeFollow: L=左エッジ, R=右エッジ */

  /* === ② 黄/青/赤共通（右旋回後の遅い直進） === */
  static constexpr int      RouteFinalApproachLeftPower  = 50;    /* 右旋回後直進 左パワー */
  static constexpr int      RouteFinalApproachRightPower = 50;    /* 右旋回後直進 右パワー */
  static constexpr int32_t  RouteFinalApproachDeg        = 380;   /* 右旋回後の直進距離 [deg] */
  /* ヨー角 PI（設置前の遅い直進）。R / L */
  static constexpr float    RouteFinalApproachBalanceKpR = 3.0f;
  static constexpr float    RouteFinalApproachBalanceKpL = 3.0f;
  static constexpr float    RouteFinalApproachBalanceKiR = 0.01f;
  static constexpr float    RouteFinalApproachBalanceKiL = 0.01f;
  static constexpr float    RouteFinalApproachBalanceIntegralMaxR = 50.0f;
  static constexpr float    RouteFinalApproachBalanceIntegralMaxL = 50.0f;
  static constexpr float    RouteFinalApproachBalanceKp =
      ETROBO_COURSE_L ? RouteFinalApproachBalanceKpR
                      : RouteFinalApproachBalanceKpL;
  static constexpr float    RouteFinalApproachBalanceKi =
      ETROBO_COURSE_L ? RouteFinalApproachBalanceKiR
                      : RouteFinalApproachBalanceKiL;
  static constexpr float    RouteFinalApproachBalanceIntegralMax =
      ETROBO_COURSE_L ? RouteFinalApproachBalanceIntegralMaxR
                      : RouteFinalApproachBalanceIntegralMaxL;

  /* ヨー角 PI（最後の後退。黄/青/赤のライン待ち後退と赤の1回目エンコーダ後退） */
  static constexpr float    RouteFinalBackBalanceKpR = 7.0f;
  static constexpr float    RouteFinalBackBalanceKpL = 7.0f;
  static constexpr float    RouteFinalBackBalanceKiR = 0.23f;
  static constexpr float    RouteFinalBackBalanceKiL = 0.3f;
  static constexpr float    RouteFinalBackBalanceIntegralMaxR = 50.0f;
  static constexpr float    RouteFinalBackBalanceIntegralMaxL = 50.0f;
  static constexpr float    RouteFinalBackBalanceKp =
      ETROBO_COURSE_L ? RouteFinalBackBalanceKpR : RouteFinalBackBalanceKpL;
  static constexpr float    RouteFinalBackBalanceKi =
      ETROBO_COURSE_L ? RouteFinalBackBalanceKiR : RouteFinalBackBalanceKiL;
  static constexpr float    RouteFinalBackBalanceIntegralMax =
      ETROBO_COURSE_L ? RouteFinalBackBalanceIntegralMaxR
                      : RouteFinalBackBalanceIntegralMaxL;
  /* === ② 黄ルート（青①検知後） === */
  static constexpr int      YellowFinalBackPower        = 50;   /* 黄到達後の後退パワー */
  static constexpr uint32_t YellowFinalBackIgnoreMs     = 400U;
  /* 黄到達後、後退を始めてからライン判定を無視する時間 [ms] */

  /* === ② 青/黄/赤共通（少し直進 → 右旋回） === */
  static constexpr int      BlueAfterSecondBlueForwardPower = 50;
  /* BlueAfterSecondBlueForwardPower: 少し直進のパワー */
  static constexpr int32_t  BlueAfterSecondBlueForwardDeg   = 160;
  /* BlueAfterSecondBlueForwardDeg: 少し直進の距離 [deg] */
  static constexpr float    BlueTurnRightOffsetDeg =
      ETROBO_COURSE_L ? -90.0f : 90.0f;
  /* BlueTurnRightOffsetDeg: 直進後の右旋回量 [deg]（現在角 + この値） */
  static constexpr float    BlueTurnToleranceDeg            = 4.0f; /* 旋回角度許容 [deg] */
  static constexpr int      BlueTurnLeftPower               = 40;   /* 旋回 左パワー */
  static constexpr int      BlueTurnRightPower              = 40;   /* 旋回 右パワー */

  /* === ② 青ルート（右旋回後） === */
  static constexpr int      BlueFinalBackPower                = 50;   /* 青③到達後の後退パワー */
  static constexpr uint32_t BlueFinalBackIgnoreMs             = 400U;
  /* 青③到達後、後退を始めてからライン判定を無視する時間 [ms] */

  /* === ② 赤ルート（設置後: エンコーダ後退 → その場旋回 → ラインまで後退 → 他分岐と同じ旋回+LT） === */
  static constexpr int      RedFinalBackPower    = 50;    /* 後退パワー（1回目・2回目共通） */
  /* 1回目後退の距離 [deg]。左右エンコーダの絶対値平均。360deg ≒ 176mm */
  static constexpr int32_t  RedFirstBackDegR     = 130;
  static constexpr int32_t  RedFirstBackDegL     = 130;
  static constexpr int32_t  RedFirstBackDeg =
      ETROBO_COURSE_L ? RedFirstBackDegR : RedFirstBackDegL;
  static constexpr uint32_t RedFinalBackIgnoreMs = 200U;
  /* 2回目後退の開始直後、ライン判定を無視する時間 [ms] */
  /* 1回目後退後のその場旋回。絶対角 [deg]（reset 後 0° 基準、CW 正。左 = 負） */
  static constexpr float    RedAfterBackTurnDegR = 50.0f;
  static constexpr float    RedAfterBackTurnDegL = -50.0f;
  static constexpr float    RedAfterBackTurnDeg =   
      ETROBO_COURSE_L ? RedAfterBackTurnDegR : RedAfterBackTurnDegL;
  static constexpr float    RedAfterBackTurnToleranceDeg = 4.0f;
  static constexpr int      RedAfterBackTurnLeftPower    = 40;
  static constexpr int      RedAfterBackTurnRightPower   = 40;
  /* ラインつかみ後、終端旋回の前にほんの少し直進する距離 [deg]（360deg ≒ 176mm） */
  static constexpr int      RedAfterLineForwardPower = 40;
  static constexpr int32_t  RedAfterLineForwardDeg   = 50;
  /* ラインつかみ後の終端旋回。黄/青の AfterFinalTurnDeg とは別に設定する */
  static constexpr float    RedAfterFinalTurnDegR = 90.0f;
  static constexpr float    RedAfterFinalTurnDegL = -90.0f;
  static constexpr float    RedAfterFinalTurnDeg =
      ETROBO_COURSE_L ? RedAfterFinalTurnDegR : RedAfterFinalTurnDegL;

  /* === ② 後退でライン検知（黄/青/赤共通） === */
  static constexpr int32_t  AfterFinalBackReflectionLineMax     = 30;
  /* AfterFinalBackReflectionLineMax: 反射光の黒線判定上限 */

  /* === ② ライン停止後: 絶対角へ旋回 → 低速LT → 白で停止 === */
  /* L と R を鏡像にせず、それぞれ独立に設定する。
   * 目標角は絶対角 [deg]（ジャイロ基準・CW 正）。
   * パワーは左右モータへそのまま入る値（正 = 前進, 負 = 後退）。
   * 旋回方向から符号を付け直さないので、回る向きはパワーの符号だけで
   * 決まる。試合前にこの 6 個だけ調整する。 */
  /* R コース（ETROBO_COURSE_L が 1）: 右輪を止めた右ピボット */
  static constexpr float    AfterFinalTurnDegR        =  90.0f;
  static constexpr int      AfterFinalTurnLeftPowerR  =  70;
  static constexpr int      AfterFinalTurnRightPowerR =   0;
  /* L コース（ETROBO_COURSE_L が 0）: 左輪を止めた左ピボット */
  static constexpr float    AfterFinalTurnDegL        = -90.0f;
  static constexpr int      AfterFinalTurnLeftPowerL  =   0;
  static constexpr int      AfterFinalTurnRightPowerL =  60;

  /* 実際に使う値。上のどちらかが選ばれるだけなので、ここは触らない。 */
  static constexpr float    AfterFinalTurnTargetDeg =
      ETROBO_COURSE_L ? AfterFinalTurnDegR : AfterFinalTurnDegL;
  static constexpr int      AfterFinalTurnLeftPower =
      ETROBO_COURSE_L ? AfterFinalTurnLeftPowerR : AfterFinalTurnLeftPowerL;
  static constexpr int      AfterFinalTurnRightPower =
      ETROBO_COURSE_L ? AfterFinalTurnRightPowerR : AfterFinalTurnRightPowerL;
  static constexpr float    AfterFinalTurnToleranceDeg = 4.0f;
  static constexpr float    FinalSlowTracePower             = 34.0f;
  static constexpr float    FinalSlowTraceKp                = 0.40f;
  static constexpr float    FinalSlowTraceKi                = 0.0001f;
  static constexpr float    FinalSlowTraceKd                = 0.8f;
  static constexpr float    FinalSlowTraceOutMax            = 100.0f;
  static constexpr float    FinalSlowTraceScale             = 1.0f;
  /* 黄: 最終LTは先にこのパワー、その後 FinalSlowTracePower へ落とす */
  static constexpr float    FinalYellowStartTracePower      = 45.0f;
  static constexpr uint32_t FinalYellowStartTraceMs         = 500U;
  /* 赤ルート時の最終LT（従来の低速） */
  static constexpr float    FinalRedSlowTracePower          = 34.0f;
  static constexpr float    FinalRedSlowTraceKp             = 0.4f;
  static constexpr float    FinalRedSlowTraceKi             = 0.0001f;
  static constexpr float    FinalRedSlowTraceKd             = 0.8f;
  static constexpr float    FinalRedSlowTraceOutMax         = 100.0f;
  static constexpr float    FinalRedSlowTraceScale          = 1.0f;
  static constexpr bool     FinalSlowTraceEdgeFollow = (ETROBO_COURSE_L == 0);
  /* FinalSlowTraceEdgeFollow: L=左エッジ, R=右エッジ */
  static constexpr uint32_t FinalSlowTraceWhiteIgnoreMs     = 700U;
  /* FinalSlowTraceWhiteIgnoreMs: 最終旋回後LT開始直後、白判定を無視する時間 [ms] */
  /* 反射光がこの値以上なら白（ColorLAB::WhiteReflectionMin と同じ系統） */
  static constexpr int32_t  FinalSlowTraceWhiteReflectionMin = 71;
};

#ifdef __cplusplus
extern "C" {
#endif

void Nansho2Task(intptr_t exinf);

#ifdef __cplusplus
}
#endif

#endif  // TESTAPP_NANSHO2_H_
