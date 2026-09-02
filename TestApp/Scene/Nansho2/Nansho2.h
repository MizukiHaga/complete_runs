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
 *  ① 赤/黄/青いずれも同じ: LT(±77°到達) → 77°で時間制御直進 → その場旋回(±90°)
 *     → ±90°で時間制御直進（黒線手前） → 左向き旋回 → 低速LT → LT → SKYBLUEまで → ジャイロreset
   *  ② 青(SKYBLUE)検知後（分岐フラグでルート選択。センサ値は保持しない）
 *     黄: 少し直進 → 右旋回 → 時間制御直進 → ジャイロreset → 後退 → 絶対±180° → 直進（黒または青検知まで）
 *     青: 黒LT → 青LT → 少し直進 → 右旋回 → 時間制御直進 → ジャイロreset → 後退 → 絶対±180° → 直進（黒または青検知まで）
 *     赤: 黒LT → 青LT → 黒LT → 青LT → 少し直進 → 右旋回 → 時間制御直進 → ジャイロreset → 後退 → 絶対±180° → 直進（黒または青検知まで）
 *     → 共通終端: 絶対角旋回(L=-90°) → 低速LT → 白で停止 */
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
  void runStraightAfterTurn(float heading, int left_power, int right_power,
                            uint32_t ms);
  void runRouteFinalBack(float heading, int power, uint32_t ms);
  void runAfterFinalBackTurnToBlue();
  void runAfterFinalStopTurnRightAndSlowTraceToWhite();
  void driveStraightUntilReflectionBlackLine(float target_heading,
                                               uint32_t ignore_ms);

  void turnToHeading(float target_heading, float tolerance_deg,
                     int left_power, int right_power);
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
  static constexpr int      ArmRaisePower = 100;     /* 上げパワー */
  static constexpr int      ArmLowerPower = -100;  /* 下げパワー */
  static constexpr int32_t  ArmRaiseDeg   = 140;   /* 上げ角度 [deg] */
  /* 0 位置を過ぎて余分に下げる量 [deg]（確実に接地） */
  static constexpr int32_t  ArmLowerOvershootDeg = 25;
  static constexpr uint32_t ArmRaiseMaxMs = 4000U; /* 上げタイムアウト [ms] */
  static constexpr uint32_t ArmLowerMaxMs = 4000U; /* 下げタイムアウト [ms] */

  /* === ⓪ ブロック色検知（0° 直進） === */
  static constexpr int      ColorForwardLeftPower  = 35;     /* 直進左パワー */
  static constexpr int      ColorForwardRightPower = 35;     /* 直進右パワー */
  static constexpr uint32_t ColorForwardMaxMs      = 3000U;  /* 色検知タイムアウト [ms] */
  /* 0° 直進のヨー角 PI 補正（gMotors.setBalancePid） */
  static constexpr float    ColorForwardBalanceKp         = 3.0f; /* P */
  static constexpr float    ColorForwardBalanceKi         = 0.01f; /* I */
  static constexpr float    ColorForwardBalanceIntegralMax = 50.0f;

  /* === ① 色検知後LT（±77°到達で終了） === */
  static constexpr float    AfterColorTraceLeftPower  = 60.0f;  /* LT 左ベースパワー */
  static constexpr float    AfterColorTraceRightPower = 60.0f; /* LT 右ベースパワー */
  static constexpr float    AfterColorTraceKp         = 0.55f;  /* LT PID: P */
  static constexpr float    AfterColorTraceKi         = 0.025f;  /* LT PID: I */
  static constexpr float    AfterColorTraceKd         = 0.8f;   /* LT PID: D */
  static constexpr float    AfterColorTraceOutMax     = 100.0f; /* LT PID 出力上限 */
  static constexpr float    AfterColorTraceScale      = 1.0f;
  static constexpr bool     AfterColorEdgeFollow      = (ETROBO_COURSE_L == 0);
  /* AfterColorEdgeFollow: L=右エッジ, R=左エッジ */
  static constexpr float    AfterColorTraceStopHeadingDeg =
      ETROBO_COURSE_L ? -77.0f : 77.0f;  /* LT 終了の目標ヨー角 [deg] */
  static constexpr float    AfterColorTraceStopToleranceDeg = 2.0f;
  /* AfterColorTraceStopToleranceDeg: LT 終了の角度許容 ±[deg] */

  /* === ① 77°到達後の直進 === */
  static constexpr int      AfterColorStraightLeftPower  = 100;  /* 直進左パワー */
  static constexpr int      AfterColorStraightRightPower = 100;  /* 直進右パワー */
  static constexpr uint32_t AfterColorStraightMs         = 300U; /* 直進時間 [ms] */

  /* === ① その場旋回（±90°） === */
  static constexpr float    AfterPivotTurnDeg =
      ETROBO_COURSE_L ? -90.0f : 90.0f;  /* 旋回目標角 [deg] */
  static constexpr float    AfterPivotTurnToleranceDeg    = 1.64f; /* 角度許容 ±[deg] */
  static constexpr int      AfterPivotTurnLeftPower       = 0;   /* 旋回 左パワー */
  static constexpr int      AfterPivotTurnRightPower      = 36;   /* 旋回 右パワー */

  /* === ① ±90° 目標で時間制御直進 → 台形減速（黒線手前） === */
  static constexpr float    AfterStraightHeadingDeg = AfterPivotTurnDeg;
  /* 直進の目標ヨー角 [deg] */
  static constexpr int      AfterStraightToBlackLeftPower  = 100;    /* 直進左パワー */
  static constexpr int      AfterStraightToBlackRightPower = 100;    /* 直進右パワー */
  static constexpr float    AfterStraightBalanceKp         = 1.5f; /* 直進補正: P */
  static constexpr float    AfterStraightBalanceKi         = 0.3f; /* 直進補正: I */
  static constexpr float    AfterStraightBalanceIntegralMax = 50.0f;
  static constexpr uint32_t AfterStraightToBlackCruiseMs   = 1225U; /* クルーズ時間 [ms] */
  static constexpr uint32_t AfterStraightToBlackDecelMs    = 500U;  /* 台形減速時間 [ms] */
  static constexpr uint32_t AfterStraightToBlackDecelStepMs = 40U;

  /* === ① 黒到達後の左向き旋回 === */
  static constexpr float    AfterBlackLeftTurnDeg =
      ETROBO_COURSE_L ? -180.0f : 180.0f;  /* 左向き旋回の目標角 [deg] */
  static constexpr float    AfterBlackLeftTurnToleranceDeg = 1.64f; /* 角度許容 [deg] */
  static constexpr int      AfterBlackLeftTurnLeftPower    = 37;   /* 旋回 左パワー（調整用） */
  static constexpr int      AfterBlackLeftTurnRightPower   = 37;   /* 旋回 右パワー（調整用） */

  /* === ① 左向き旋回直後の低速LT（線に乗せる） === */
  static constexpr float    PostBlackSlowTracePower  = 35.0f; /* 低速LT パワー */
  static constexpr uint32_t PostBlackSlowTraceMs     = 600U;  /* 低速LT 時間 [ms] */
  static constexpr float    PostBlackSlowTraceKp     = 0.30f;
  static constexpr float    PostBlackSlowTraceKi     = 0.01f;
  static constexpr float    PostBlackSlowTraceKd     = 0.8f;
  static constexpr float    PostBlackSlowTraceOutMax = 100.0f;
  static constexpr float    PostBlackSlowTraceScale  = 1.0f;

  /* === ① 左向き旋回後LT（SKYBLUE①まで） === */
  static constexpr float    PostBlackTraceLeftPower  = 50.0f;  /* LT 左ベースパワー */
  static constexpr float    PostBlackTraceRightPower = 50.0f; /* LT 右ベースパワー */
  static constexpr float    PostBlackTraceKp         = 0.45;  /* LT PID: P */
  static constexpr float    PostBlackTraceKi         = 0.01f;   /* LT PID: I */
  static constexpr float    PostBlackTraceKd         = 0.8f;   /* LT PID: D */
  static constexpr float    PostBlackTraceOutMax     = 100.0f; /* LT PID 出力上限 */
  static constexpr float    PostBlackTraceScale      = 1.0f;
  static constexpr bool     PostBlackTraceEdgeFollow = (ETROBO_COURSE_L != 0);
  /* PostBlackTraceEdgeFollow: L=左エッジ, R=右エッジ */

  /* === ② 黄/青/赤共通（右旋回後の直進） === */
  static constexpr int      RouteFinalApproachLeftPower  = 50;    /* 右旋回後直進 左パワー */
  static constexpr int      RouteFinalApproachRightPower = 50;    /* 右旋回後直進 右パワー */
  static constexpr uint32_t RouteFinalApproachMs         = 450U;  /* 右旋回後の直進時間 [ms] */
  /* === ② 黄ルート（青①検知後） === */
  static constexpr int      YellowFinalBackPower        = 50;   /* 黄到達後の後退パワー */
  static constexpr uint32_t YellowFinalBackMs           = 800U;
  /* 黄到達後の後退時間 [ms] */

  /* === ② 青/黄/赤共通（少し直進 → 右旋回） === */
  static constexpr int      BlueAfterSecondBlueForwardPower = 50;
  /* BlueAfterSecondBlueForwardPower: 少し直進のパワー */
  static constexpr uint32_t BlueAfterSecondBlueForwardMs    = 200U;
  /* BlueAfterSecondBlueForwardMs: 少し直進の時間 [ms] */
  static constexpr float    BlueTurnRightOffsetDeg =
      ETROBO_COURSE_L ? 90.0f : -90.0f;
  /* BlueTurnRightOffsetDeg: 直進後の右旋回量 [deg]（現在角 + この値） */
  static constexpr float    BlueTurnToleranceDeg            = 4.0f; /* 旋回角度許容 [deg] */
  static constexpr int      BlueTurnLeftPower               = 40;   /* 旋回 左パワー */
  static constexpr int      BlueTurnRightPower              = 40;   /* 旋回 右パワー */

  /* === ② 青ルート（右旋回後） === */
  static constexpr int      BlueFinalBackPower                = 50;   /* 青③到達後の後退パワー */
  static constexpr uint32_t BlueFinalBackMs                   = 800U; /* 青③到達後の後退時間 [ms] */

  /* === ② 赤ルート（右旋回後） === */
  static constexpr int      RedFinalBackPower                 = 50;   /* 赤到達後の後退パワー */
  static constexpr uint32_t RedFinalBackMs                    = 800U; /* 赤到達後の後退時間 [ms] */

  /* === ② 後退後（黄/青/赤共通）: 絶対±180°旋回 → 直進（黒または青検知まで） === */
  static constexpr float    AfterFinalBackTurnDeg =
      ETROBO_COURSE_L ? 180.0f : -180.0f;
  /* AfterFinalBackTurnDeg: 直進後 reset 基準の絶対目標角 [deg] */
  static constexpr float    AfterFinalBackTurnToleranceDeg    = 1.8f; /* 旋回角度許容 [deg] */
  static constexpr int      AfterFinalBackTurnLeftPower     = 35;   /* 旋回 左パワー */
  static constexpr int      AfterFinalBackTurnRightPower    = 40;   /* 旋回 右パワー */
  static constexpr int      AfterFinalBackBlueApproachLeftPower  = 50; /* 直進 左パワー */
  static constexpr int      AfterFinalBackBlueApproachRightPower = 50; /* 直進 右パワー */
  static constexpr int32_t  AfterFinalBackReflectionLineMax     = 30;
  /* AfterFinalBackReflectionLineMax: 反射光の黒線判定上限 */
  static constexpr uint32_t AfterFinalBackBlueIgnoreMs        = 0U;
  /* AfterFinalBackBlueIgnoreMs: 直進開始後、黒/青判定を無視する時間 [ms] */

  /* === ② ライン停止後: 絶対角へ旋回 → 低速LT → 白で停止 === */
  static constexpr float    AfterFinalTurnToZeroDeg =
      ETROBO_COURSE_L ? -90.0f : 0.0f;
  /* AfterFinalTurnToZeroDeg: ライン停止後の絶対目標角 [deg]（L=-90 / R=0） */
  static constexpr float    AfterFinalRightTurnToleranceDeg = 4.0f;
  static constexpr int      AfterFinalRightTurnLeftPower    = 50;
  static constexpr int      AfterFinalRightTurnRightPower   = 40;
  static constexpr float    FinalSlowTracePower             = 50.0f;
  static constexpr float    FinalSlowTraceKp                = 0.30f;
  static constexpr float    FinalSlowTraceKi                = 0.01f;
  static constexpr float    FinalSlowTraceKd                = 0.8f;
  static constexpr float    FinalSlowTraceOutMax            = 100.0f;
  static constexpr float    FinalSlowTraceScale             = 1.0f;
  static constexpr bool     FinalSlowTraceEdgeFollow = (ETROBO_COURSE_L != 0);
  /* FinalSlowTraceEdgeFollow: L=左エッジ, R=右エッジ */
  static constexpr uint32_t FinalSlowTraceWhiteIgnoreMs     = 800U;
  /* FinalSlowTraceWhiteIgnoreMs: 最終旋回後LT開始直後、白判定を無視する時間 [ms] */
};

#ifdef __cplusplus
extern "C" {
#endif

void Nansho2Task(intptr_t exinf);

#ifdef __cplusplus
}
#endif

#endif  // TESTAPP_NANSHO2_H_
