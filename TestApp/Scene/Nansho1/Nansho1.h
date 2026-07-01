#ifndef TESTAPP_NANSHO1_H_
#define TESTAPP_NANSHO1_H_

#include <kernel.h>
#include <pbio/port.h>

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
   *  ① ライントレースでバックし円の終端まで移動 (固定時間 BackToEndMs)
   *  ② 円の方向へ旋回 (右 90°)、旋回後の向きを 0° にリセット、
   *     指定時間 (ForwardBeforeScanMs) 前進
   *  ③ 右 90° 走査: 検知 → 灰色直進 → 待機・バック → 180° → 黒線まで直進 → ⑦
   *  ④ 検知なしで規定回数完了 → 円の反対を読み取る動き
   *       (readOppositeSide: 直進 → 右旋回 → 直進 → 右旋回 → 灰色まで直進)
   *       旧: 旋回中に超音波検知 → finishAfterDetect(Left, +180°) は廃止
   *  ⑤ 左 90° 走査: 検知 → 灰色直進 → 待機・バック → finishAfterDetect (角度で分岐) → ⑦
   *  ⑥ 検知なしで規定回数完了 → 黒線を読む動き → ⑦
   *  ⑦ 黒線読み後: 左 90° 旋回 → 青線までライントレース
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
    SonicDetected,  /* 超音波で DetectMaxMm 以下を検知 */
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
  ScanResult wigwagScan(float amp_deg, int max_cycles, float *out_heading);

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

  /* 旋回しながら超音波だけ監視 (黒線は見ない)。 */
  ScanResult rotateToHeadingWithScan(float target_heading, float *out_heading);

  /* 直進しながら超音波 + 黒線エッジを監視。 */
  ScanResult driveForwardWithScan(uint32_t duration_ms, int power, float *out_heading);

  /* 現在のヨー角を target_heading に合わせるピボット旋回 (検知判定なし)。
   *   tolerance_deg: 目標角到達とみなす許容幅 [deg] (省略時 TurnToleranceDeg)。 */
  void turnToHeading(float target_heading,
                     float tolerance_deg = TurnToleranceDeg);

  /* 左右モーター個別パワーでピボット旋回 (符号は旋回方向に合わせて付与)。
   *   left_power / right_power: 正のパワー値 (CW 時 L=+, R=− / CCW 時 L=−, R=+)。 */
  void turnToHeading(float target_heading, float tolerance_deg,
                     int left_power, int right_power);

  /* その場で 360° ピボット旋回しながら超音波を読み続ける (即時停止式)。 */
  bool scan360(float *out_heading);

   /* 開始時: ライントレース (スカイブルー → 黒) → 90° 旋回。 */
   void runInitialForwardTrace();

  /* 最終黒線読み後: 左 90° 旋回 → 青線までライントレース。 */
  void runAfterFinalBlackLine();

  /* === ⓪ 開始時 (ライントレース) パラメータ */
  static constexpr float    InitialTracePower   = 33.0f;
  static constexpr float    InitialBlackTurnDeg = 180.0f;

  /* === ⑦ 最終黒線読み後 (青線ライントレース) パラメータ */
  static constexpr float    FinalBlueTraceTurnDeg        = 90.0f;
  /* 黒線読み後の旋回: 左右モーターパワー (正値、方向は自動で付与)。 */
  static constexpr int      FinalBlueTraceTurnLeftPower  = 30;
  static constexpr int      FinalBlueTraceTurnRightPower = 35;
  static constexpr float    FinalBlueTracePower          = 80.0f;

  /* === ① バック・ライントレース (円の終端まで) パラメータ ========= */
  /* バック・ライントレースする時間 [ms]。 */
  static constexpr uint32_t BackToEndMs        = 2220U;
  /* バック・ライントレースの左右パワー (負値で後退)。 */
  static constexpr int      BackTracePower     = 100;
  /* バック LT 中の減速: DecelMs かけて Power → EndPower へ線形に下げる。 */
  static constexpr int      BackTraceEndPower  = 50;
  static constexpr uint32_t BackTraceDecelMs   = 2000U;

  /* === ② 旋回後の前進 (走査開始前) パラメータ =============== */
  /* 右 90° 旋回 + ヨー角リセットのあと、走査前に前進する時間 [ms]
   * (パワーは LeftPower / RightPower を使用)。 */
  static constexpr uint32_t ForwardBeforeScanMs = 1000U;

  /* === 検知共通パラメータ ==================================== */
  /* 物体を「検知あり」とみなす最大距離 [mm]。
   * ジグザグスキャン中・360° スキャン中・前進中いずれでも共通。 */
  static constexpr int32_t  DetectMaxMm        = 180;

  /* === 旋回走査 (wigwag) パラメータ ========================== */
  /* 1 方向あたりの「規定回数」サイクル数。９
   * これだけ繰り返して検知が無ければ次の動作へ移る。 */
  static constexpr int      ScanCycles         = 5;
  /* 振る角度 [°] (右走査は +、左走査は -)。 */
  static constexpr float    WigWagAmpDeg       = 90.0f;
  /* 1 サイクル末の「少し直進」する時間 [ms] とパワー。 */
  static constexpr uint32_t WigWagForwardMs    = 600U;
  static constexpr int      WigWagForwardPower = 40;

  /* === 360° スキャンフェーズ用パラメータ ===================== */
  /* ピボット旋回パワー (左右逆符号で同値、その場旋回) */
  static constexpr int      ScanTurnPower          = 40;
  /* 走査旋回 (rotateToHeadingWithScan): 目標角到達とみなす許容幅 [deg]。 */
  static constexpr float    ScanTurnToleranceDeg     = 1.8f;
  /* スキャンを打ち切る最大経過時間 [ms] (安全用) */
  static constexpr uint32_t ScanMaxMs              = 15000U;

  /* === 目標方向への旋回パラメータ ============================ */
  static constexpr int      TurnPower          = 40;
  static constexpr float    TurnToleranceDeg   = 1.8f;
  /* turnToHeading の打ち切り時間 [ms] (安全用) */
  static constexpr uint32_t TurnMaxMs          = 8000U;

  /* === 円の反対を読み取る動き (readOppositeSide) パラメータ === */
  static constexpr int      OppositeForwardPower       = 50;
  /* 1 回目の直進時間 [ms]。 */
  static constexpr uint32_t OppositeForwardFirstMs   = 1200U;
  /* 右旋回後の 2 回目の直進時間 [ms]。 */
  static constexpr uint32_t OppositeForwardSecondMs  = 1250U;
  static constexpr int      OppositeTurnPower          = 40;
  /* 各「右を向く」ときの旋回角 [deg] (turnToHeading の目標角)。 */
  static constexpr float    OppositeTurnRightDeg       = 90.0f;
  static constexpr float    OppositeTurnToleranceDeg   = 1.8f;

  /* === 検知後の終了動作 (finishAfterDetect) パラメータ ========= */
  /* 超音波検知後の灰色直進: 開始直後この時間 [ms] は灰色判定を無視する。 */
  static constexpr uint32_t DetectGrayIgnoreMs    = 500U;
  /* 検知後共通: 灰色停止からバック開始までの待ち時間 [ms] (慣性で後部が上がるのを防ぐ)。 */
  static constexpr uint32_t DetectFinishBackDelayMs = 500U;
  /* 検知後共通: 灰色直進後のバック時間 [ms] / パワー。 */
  static constexpr uint32_t DetectFinishBackMs    = 1000U;
  static constexpr int      DetectFinishBackPower = 50;
  /* 右走査検知後: バックのあと向く角度 [deg]。 */
  static constexpr float    DetectFinishRightTurnDeg = 180.0f;
  /* 左走査検知後 (-56° 分岐): この角度 [deg] 未満なら 0° 向き黒線へ。 */
  static constexpr float    DetectFinishClassicMaxDeg = -19.0f;
  /* 左走査検知後: この角度 [deg] 未満なら広い検知角ルートへ。 */
  static constexpr float    DetectFinishWideMaxDeg    = -20.0f;
  static constexpr float    DetectWideTurnFirstDeg    = -90.0f;
  /* 中間直進のあとに向く角度 [deg]。 */
  static constexpr float    DetectWideTurnSecondDeg   = 90.0f;
  /* 広い検知角ルート: -90° 旋回後の直進時間 [ms]。 */
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
