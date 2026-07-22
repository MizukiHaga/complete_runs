#ifndef TESTAPP_NANSHO1_H_
#define TESTAPP_NANSHO1_H_

#include <kernel.h>
#include <pbio/port.h>

#include "app.h"
#include "ColorLAB.h"
#include "ColorRGB.h"
#include "Gyro.h"
#include "Sonic.h"
#include "Straight.h"

#define NANSHO1_ULTRASONIC_PORT PBIO_PORT_ID_F

class Nansho1Scene {
public:
  /* === シーンの流れ ===========================================
   *  ⓪ 開始時: ライントレース (スカイブルー → 黒) → 90° 旋回
   *  ① 旋回後に低速 LT → 本線 LT で円の終端まで移動 (固定時間 BackToEndMs)
   *  ② 円の方向へ旋回 (右 90°)、旋回後の向きを 0° にリセット、
   *     指定時間 (ForwardBeforeScanMs) 前進
   *  ③ 右 90° 走査: 検知 → 絶対 +90° 向き → 黒線まで直進 → ⑦
   *  ④ 検知なしで規定回数完了 → 円の反対を読み取る動き
   *       (readOppositeSide: 直進 → 右旋回 → 直進 → 右旋回 → 灰色まで直進)
   *  ⑤ 左 90° 走査: 検知 → 絶対 +90° 向き → 黒線まで直進 → ⑦
   *  ⑥ 検知なしで規定回数完了 → 黒線を読む動き → ⑦
   *  ⑦ 黒線読み後: 指定方向に 90° 旋回 → 青線までライントレース
   * ============================================================ */
  void run();

  /* === 公開パラメータ ======================================== */
  /* 黒線到達まで直進するときの左右パワー (機械差吸収のため別値) */
  static constexpr int LeftPower  = 50;
  static constexpr int RightPower = 50;

private:
  /* 走査方向 (検知後の終了動作分岐用) */
  enum class ScanSide {
    Right,  /* wigwagScan(+WigWagAmpDeg) */
    Left,   /* wigwagScan(-WigWagAmpDeg) */
  };

  enum class ScanResult {
    Completed,      /* rotate: 目標角到達 / drive: 時間経過 / wigwag: 規定回数完了 */
    SonicDetected,  /* 超音波距離で障害物を検知 */
    BlackDetected,  /* 前進中に黒線エッジを踏んだ */
  };

  /* 旋回走査 (片側振り・規定回数で打ち切り)。
   *   amp_deg    : 振り角 (右走査は +WigWagAmpDeg、左走査は -WigWagAmpDeg)
   *   max_cycles : 「規定回数」サイクル数
   *   返り値:
   *     SonicDetected → out_heading に検知時ヨー角
   *     BlackDetected → 前進中に黒線エッジを踏んだ
   *     Completed     → max_cycles を検知なしで完了
   *
   *   黒線の判定は前進フェーズでのみ行う。
   *   その場ピボット旋回中はカラーセンサが弧を描いて元の黒線を
   *   横切り直してしまい誤終了するため、旋回中は黒を見ない。 */
  ScanResult wigwagScan(float amp_deg, int max_cycles, float *out_heading,
                        ScanSide scan_side);

  /* 円の反対を読み取る動き: 直進 → 右旋回 → 直進 → 右旋回 → 灰色まで直進。 */
  ScanResult readOppositeSide();

  /* 黒線を読む動き: target_heading を向いてから黒線エッジを踏むまで前進。 */
  void driveUntilBlackLine(float target_heading = 0.0f);

  /* 灰色ラインを踏むまで直進して停止。
   * gray_ignore_ms: 開始直後この時間 [ms] は灰色判定を無視 (0 で無効)。 */
  void driveUntilGrayLine(uint32_t gray_ignore_ms = DetectGrayIgnoreMs,
                          int left_power = LeftPower,
                          int right_power = RightPower);

  /* 検知後の終了動作 (走査方向で分岐)。灰色直進後に待機・バックしてから分岐。
   *   heading_offset_deg: Left 分岐の旋回目標角に加算 (④ 旋回中検知時は +180°)
   *   Right: 180° → 黒線まで直進
   *   Left:  heading < -56° → (0°+offset) 向き黒線 / 以上 → (-90°+offset) → 直進 → (90°+offset) → 黒線 */
  void finishAfterDetect(ScanSide side, float heading_offset_deg = 0.0f);

  /* 現在の向きのまま黒線エッジを踏むまで前進して停止。 */
  void driveForwardUntilBlackLine();

  /* 旋回しながら超音波だけ監視 (黒線は見ない)。
   *   tolerance_deg: 目標角到達とみなす許容幅 [deg] */
  ScanResult rotateToHeadingWithScan(float target_heading, float *out_heading,
                                     ScanSide scan_side, float tolerance_deg);

  /* 直進しながら超音波 + 黒線エッジを監視。 */
  ScanResult driveForwardWithScan(uint32_t duration_ms, int power, float *out_heading);

  /* 現在のヨー角を target_heading に合わせるピボット旋回 (検知判定なし)。
   *   tolerance_deg: 目標角到達とみなす許容幅 [deg] (省略時 TurnToleranceDeg)。 */
  void turnToHeading(float target_heading,
                     float tolerance_deg = TurnToleranceDeg);

  /* 左右モーター個別パワーでピボット旋回 (符号は旋回方向に合わせて付与)。
   *   left_power / right_power: 正のパワー値 (CW 時 L=+, R=− / CCW 時 L=−, R=+)。
   *   reset_after: true なら停止後に gGyro.reset() する。 */
  void turnToHeading(float target_heading, float tolerance_deg,
                     int left_power, int right_power,
                     bool reset_after = true);

  /* 超音波検知後: DetectAfterSonicTurnDeg を向いて停止 → 0° に向き直す。 */
  void faceDetectHeadingThenReturnToZero();

  /* その場で 360° ピボット旋回しながら超音波を読み続ける (即時停止式)。 */
  bool scan360(float *out_heading);

  /* ③ 1回目 wig走査 → ④ 反対読み → ⑤ 2回目 wig走査。検知時 true。 */
  bool runWigwagScanPhases();

   /* 開始時: ライントレース (スカイブルー → 黒) → 90° 旋回。 */
   void runInitialForwardTrace();

  /* 最終黒線読み後: 指定方向に 90° 旋回 → 青線までライントレース。 */
  void runAfterFinalBlackLine();
  void turnAfterFinalBlackLine();

  /* === ⓪ 開始時 (ライントレース) パラメータ */
  static constexpr float    InitialTracePower   = 33.0f;
  static constexpr float    InitialBlackTurnDeg = 180.0f;

  /* === ⑦ 最終黒線読み後 (青線ライントレース) パラメータ */
  /* 黒線読み後の旋回方向: true=左旋回, false=右旋回。試合前にここだけ変更する。 */
  static constexpr bool     FinalBlackTurnLeft            = (ETROBO_COURSE_L == 1);
  static constexpr float    FinalBlueTraceTurnDeg           = 90.0f;
  static constexpr float    FinalBlueTraceTurnToleranceDeg  = 2.5f;
  static constexpr uint32_t FinalBlueTraceTurnMaxMs         = 8000U;
  static constexpr int      FinalBlueTraceTurnSign =
      FinalBlackTurnLeft ? -1 : 1;
  /* 黒線読み後の旋回パワー（左右を独立に調整）。
   * 値は正の大きさを入れる。符号は FinalBlackTurnLeft で自動付与。 */
  static constexpr int      FinalBlueTraceTurnLeftPowerMag  = 30;
  static constexpr int      FinalBlueTraceTurnRightPowerMag = 43;
  static constexpr int      FinalBlueTraceTurnLeftPower =
      FinalBlackTurnLeft ? -FinalBlueTraceTurnLeftPowerMag : FinalBlueTraceTurnLeftPowerMag;
  static constexpr int      FinalBlueTraceTurnRightPower =
      FinalBlackTurnLeft ? FinalBlueTraceTurnRightPowerMag : -FinalBlueTraceTurnRightPowerMag;
  static constexpr float    FinalBlueTracePower             = 80.0f;

  /* === ① バック・ライントレース (円の終端まで) パラメータ ========= */
  /* 旋回直後の低速 LT (本線 LT の前に線へ乗せる)。 */
  static constexpr int      BackSlowTracePower = 35;
  static constexpr uint32_t BackSlowTraceMs    = 600U;
  /* バック・ライントレースする時間 [ms]。 */
  static constexpr uint32_t BackToEndMs        = 2100U;
  /* バック・ライントレースの左右パワー (負値で後退)。 */
  static constexpr int      BackTracePower     = 80;
  /* バック LT 中の減速: DecelMs かけて Power → EndPower へ線形に下げる。 */
  static constexpr int      BackTraceEndPower  = 30;
  static constexpr uint32_t BackTraceDecelMs   = 3200U;

  /* === 検知共通パラメータ ==================================== */
  /* 走査検知: 超音波距離がこの値以下なら障害物ありとみなす [mm]。 */
  static constexpr int32_t  DetectMaxMm        = 250;

  /* === 旋回走査 (wigwag) パラメータ ========================== */
  /* 1 方向あたりの「規定回数」サイクル数。９
   * これだけ繰り返して検知が無ければ次の動作へ移る。 */
  static constexpr int      ScanCycles         = 8;
  /* 振る角度 [°] (右走査は +、左走査は -)。
   * 1サイクル = 側方へ旋回 → 正面へ戻り → 少し前進 (reset なし)。 */
  static constexpr float    WigWagAmpDeg       = 90.0f;
  /* 1 サイクル末の「少し直進」する時間 [ms] とパワー。 */
  static constexpr uint32_t WigWagForwardMs    = 400U;
  static constexpr int      WigWagForwardPower = 40;
  /* 左走査の旋回中に超音波検知したとき、停止せず旋回を継続する時間 [ms]。 */
  static constexpr uint32_t LeftScanDetectContinueMs = 0U;

  /* === 360° スキャンフェーズ用パラメータ ===================== */
  /* ピボット旋回パワー (左右逆符号で同値、その場旋回) */
  static constexpr int      ScanTurnPower          = 40;
  /* 走査旋回の許容幅 [deg] (大きいほど早めに到達扱い)。 */
  /* 側方 90° への往路旋回 */
  static constexpr float    WigWagOutToleranceDeg    = 2.5f;
  /* 正面へ戻る復路旋回 */
  static constexpr float    WigWagReturnToleranceDeg = 2.5f;
  /* スキャンを打ち切る最大経過時間 [ms] (安全用) */
  static constexpr uint32_t ScanMaxMs              = 15000U;

  /* === 目標方向への旋回パラメータ ============================ */
  static constexpr int      TurnPower          = 40;
  static constexpr float    TurnToleranceDeg   = 4.5f;
  /* この角度以内になったら低速に落とす [deg]（行き過ぎ防止） */
  static constexpr float    TurnSlowApproachDeg = 30.0f;
  static constexpr int      TurnSlowPower       = 33;
  /* turnToHeading の打ち切り時間 [ms] (安全用) */
  static constexpr uint32_t TurnMaxMs          = 8000U;

  /* === 円の反対を読み取る動き (readOppositeSide) パラメータ === */
  static constexpr int      OppositeForwardPower       = 50;
  /* 1 回目の直進時間 [ms]。 */
  static constexpr uint32_t OppositeForwardFirstMs   = 700U;
  /* 右旋回後の 2 回目の直進時間 [ms]。 */
  static constexpr uint32_t OppositeForwardSecondMs  = 1500U;
  static constexpr int      OppositeTurnPower          = 45;
  /* 反対読みの旋回角 [deg] (app.h の ETROBO_COURSE_L で L/R 切り替え)。 */
  static constexpr float    OppositeTurnDeg            =
      ETROBO_COURSE_L ? 90.0f : -90.0f;
  static constexpr float    OppositeTurnToleranceDeg   = 4.0f;

  /* === ② 旋回後の前進 (走査開始前) パラメータ =============== */
  /* 円方向への旋回角 [deg] (app.h で L/R 切り替え)。 */
  static constexpr float    PreWigwagTurnDeg           =
      ETROBO_COURSE_L ? 90.0f : -90.0f;
  /* 旋回 + ヨー角リセットのあと、走査前に前進する時間 [ms]。 */
  static constexpr uint32_t ForwardBeforeScanMs = 1000U;
  static constexpr float    PreWigwagBalanceKp          = 0.35f;
  static constexpr float    PreWigwagBalanceKi          = 0.01f;
  static constexpr float    PreWigwagBalanceIntegralMax = 50.0f;

  /* === 検知後の終了動作 (finishAfterDetect) パラメータ ========= */
  /* 超音波検知後の灰色直進: 開始直後この時間 [ms] は灰色判定を無視する。 */
  static constexpr uint32_t DetectGrayIgnoreMs    = 600U;
  /* 検知後共通: 灰色停止からバック開始までの待ち時間 [ms] (慣性で後部が上がるのを防ぐ)。 */
  static constexpr uint32_t DetectFinishBackDelayMs = 300U;
  /* 検知後共通: 灰色直進後のバック時間 [ms] / パワー。 */
  static constexpr uint32_t DetectFinishBackMs    = 700U;
  static constexpr int      DetectFinishBackPower = 50;
  /* 超音波検知後: まずこの絶対角を向いて停止し、続けて 0° に向き直す [deg]。 */
  static constexpr float    DetectAfterSonicTurnDeg =
      ETROBO_COURSE_L ? 90.0f : -90.0f;
  /* 左走査検知後の角度分岐 (app.h で L/R 切り替え)。 */
  static constexpr int      DetectFinishAngleSign =
      ETROBO_COURSE_L ? 1 : -1;
  static constexpr float    DetectFinishClassicMaxDeg =
      ETROBO_COURSE_L ? -40.0f : 40.0f;
  static constexpr float    DetectWideTurnFirstDeg =
      ETROBO_COURSE_L ? -90.0f : 90.0f;
  static constexpr float    DetectWideTurnSecondDeg =
      ETROBO_COURSE_L ? 90.0f : -90.0f;
  /* 広い検知角ルート: 1回目旋回後の直進時間 [ms]。 */
  static constexpr uint32_t DetectMidForwardMs          = 1500U;

  /* === 制御ループ周期 [ms] =================================== */
  static constexpr uint32_t StepMs             = 10U;
};

#ifdef __cplusplus
extern "C" {
#endif

void Nansho1Task(intptr_t exinf);

#ifdef __cplusplus
}
#endif

#endif  // TESTAPP_NANSHO1_H_
