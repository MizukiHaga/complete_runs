#ifndef TESTAPP_NANSHO1_H_
#define TESTAPP_NANSHO1_H_

#include <kernel.h>
#include <pbio/port.h>

#include "Color.h"
#include "Gyro.h"
#include "Sonic.h"
#include "Straight.h"

#define NANSHO1_ULTRASONIC_PORT PBIO_PORT_ID_F

class Nansho1Scene {
public:
  /* === シーンの流れ ===========================================
   *  1. 黒線を踏むまで直進                       (LeftPower / RightPower)
   *  2. IMU ヨー角を 0 にリセット
   *  3. ジグザグ確認スキャン (wigwagScan):
   *       「+WigWagAmpDeg → -WigWagAmpDeg → 0° に戻る → 少し直進」を
   *       超音波 / カラーセンサを監視しながら、
   *         a) 超音波で物体検知 (DetectMaxMm 以下)             → SonicDetected
   *         b) 一度「黒以外」を踏んだ後に再び黒線を読んだ瞬間  → BlackDetected
   *       のいずれかが起きるまで繰り返す。
   *       a) は出口 4a へ、b) は出口 4b へ。
   *  4a. 検知あり: その方向へ向き直って前進
   *  4b. 黒線で終了 (または検知なし): 360° スキャン (即時停止式)
   *      → 検知方向へ向き直って前進
   * ============================================================ */
  void run();

  /* === 公開パラメータ ======================================== */
  /* 黒線到達まで直進するときの左右パワー (機械差吸収のため別値) */
  static constexpr int LeftPower  = 30;
  static constexpr int RightPower = 30;

private:
  /* ジグザグ確認スキャン (回数制限なし)。
   *   - 超音波で物体検知 → true、out_heading に検知時ヨー角
   *   - 前進中に「非黒を踏んだ後の」黒線エッジ検知 → false
   *
   *   黒線の判定は前進フェーズでのみ行う。
   *   その場ピボット旋回中はカラーセンサが弧を描いて元の黒線を
   *   横切り直してしまい誤終了するため、旋回中は黒を見ない。 */
  bool wigwagScan(float *out_heading);

  /* helper の停止理由 */
  enum class ScanResult {
    Completed,      /* rotate: 目標角到達 / drive: 時間経過 */
    SonicDetected,  /* 超音波で DetectMaxMm 以下を検知 */
    BlackDetected,  /* 前進中に黒線エッジを踏んだ */
  };

  /* 旋回しながら超音波だけ監視 (黒線は見ない)。 */
  ScanResult rotateToHeadingWithScan(float target_heading, float *out_heading);

  /* 直進しながら超音波 + 黒線エッジを監視。 */
  ScanResult driveForwardWithScan(uint32_t duration_ms, int power, float *out_heading);

  /* 現在のヨー角を target_heading に合わせるピボット旋回 (検知判定なし)。
   *   - 短い側 (|diff| <= 180) を選んで回る。
   *   - |diff| <= TurnToleranceDeg または TurnMaxMs 経過で停止。 */
  void turnToHeading(float target_heading);

  /* その場で 360° ピボット旋回しながら超音波を読み続ける (即時停止式)。 */
  bool scan360(float *out_heading);

  /* 検知方向 (heading) に向き直って ApproachTimeMs だけ前進する
   * シーン共通の終了動作。 */
  void approachHeading(float heading);

  /* === 検知共通パラメータ ==================================== */
  /* 物体を「検知あり」とみなす最大距離 [mm]。
   * ジグザグスキャン中・360° スキャン中・前進中いずれでも共通。 */
  static constexpr int32_t  DetectMaxMm        = 180;

  /* === ジグザグスキャン (wigwag) パラメータ ================== */
  /* 左右に振る角度 [°] (±この値)。 */
  static constexpr float    WigWagAmpDeg       = 90.0f;
  /* 1 サイクル末の「少し直進」する時間 [ms] とパワー。 */
  static constexpr uint32_t WigWagForwardMs    = 1000U;
  static constexpr int      WigWagForwardPower = 35;

  /* === 360° スキャンフェーズ用パラメータ ===================== */
  /* ピボット旋回パワー (左右逆符号で同値、その場旋回) */
  static constexpr int      ScanTurnPower      = 40;
  /* スキャンを打ち切る最大経過時間 [ms] (安全用) */
  static constexpr uint32_t ScanMaxMs          = 15000U;

  /* === 目標方向への旋回パラメータ ============================ */
  static constexpr int      TurnPower          = 30;
  static constexpr float    TurnToleranceDeg   = 5.0f;
  /* turnToHeading / rotateToHeadingWithScan の打ち切り時間 [ms] (安全用) */
  static constexpr uint32_t TurnMaxMs          = 8000U;

  /* === 検知方向への前進パラメータ ============================ */
  static constexpr int      ApproachPower      = 40;
  static constexpr uint32_t ApproachTimeMs     = 3000U;

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
