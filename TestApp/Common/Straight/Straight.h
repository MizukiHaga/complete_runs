/*
 * Straight モジュール ヘッダ
 *  左右モータをまとめて操作する直進系の共通モジュール。
 *  pup_motor_* の C API は Straight クラスの中だけで使い、
 *  外からは run/stop/runFor などのメソッドで操作する。
 *
 *  直進補正:
 *    左右パワーが同じ (1:1) のとき、目標ヨー角を維持するよう
 *    gGyro の角度ずれを PI 補正する。目標角は setTargetHeading() で指定
 *   （未指定時は 0°。run 開始時の現在角は掴み直さない）。
 *    弧走行・ピボットでは補正しない。
 *
 *  使い方:
 *    Straight gMotors;
 *    gGyro.reset();
 *    gMotors.configure(PBIO_PORT_ID_B, PBIO_PORT_ID_A);
 *    gMotors.setTargetHeading(0.0f);   // 任意: 目標角を明示指定
 *    gMotors.setBalancePid(0.35f, 0.01f);
 *    gMotors.run(50, 50);
 *    gMotors.run(50, 50, 720);  // 左右エンコーダ平均 720° で停止（ヨー補正あり）
 *    gMotors.stop();
 *    gMotors.runFor(50, 50, 720);  // エンコーダ終了・ヨー補正なし
 */

#ifndef TESTAPP_STRAIGHT_H_
#define TESTAPP_STRAIGHT_H_

#include <pbio/port.h>
#include <stdbool.h>
#include <stdint.h>

extern "C" {
#include <spike/pup/motor.h>
}

class Straight {
public:
  static constexpr uint32_t DecelMs = 800U;
  static constexpr uint32_t DecelStepMs = 40U;
  static constexpr uint32_t MaxDurationMs = 4000000U;

  /* === ヨー角 PI 直進補正 (既定値) ==========================
   * error = 現在角 − 目標角 [deg] (±180 に折り畳み)
   * 右 = + なので error>0 は右ずれ → 左旋回 (L−trim / R+trim)
   * 後退時も同じ式（負パワーで L−＝後退強、R+＝後退弱） */
  static constexpr float    DefaultBalanceKp   = 0.35f;
  static constexpr float    DefaultBalanceKi   = 0.01f;
  static constexpr float    BalanceIntegralMax = 50.0f;
  /* 1 周期あたりの左右パワー差 |L-R| の上限。
   * ここが小さいと Kp を上げても trim が頭打ちになり挙動が変わらない。 */
  static constexpr int      MaxHeadingTrim     = 40;

  void configure(pbio_port_id_t left_motor_port, pbio_port_id_t right_motor_port);

  /* 直進補正の目標ヨー角 [deg] を指定する (integral をリセット)。
   * 未指定時は 0°（run 開始時の現在角は使わない）。
   * stop() / coast() でも目標は残る。捨てるときは resetCorrection()。 */
  void setTargetHeading(float heading_deg);

  /* ヨー角 PI ゲインを実行時に変更する。設定時に積分項をリセット。
   * i_max: 積分項クランプ上限 [deg] (省略時 BalanceIntegralMax)。
   * deadband_deg: |error| がこの以内なら P=0・I を減衰（省略時は不感帯なし）。 */
  void setBalancePid(float kp, float ki, float i_max = BalanceIntegralMax,
                     float deadband_deg = 0.0f);

  /* シリアルに tgt/cur/err/trim/L/R を出す（調整用）。 */
  void setCorrectionDebug(bool enabled) { correction_debug_ = enabled; }

  /* ヨー角 PI 直進補正の有効/無効。false なら 1:1 直進も open-loop。
   * stop() とは別（stop 中の出力抑止は correction_enabled_ 側）。 */
  void setHeadingCorrection(bool enabled);

  /* true: 1:1 直進の補正をヨー角ではなく左右エンコーダ差にする。
   * 誤差 = 進行方向の (左進み − 右進み) [deg]。左が進みすぎなら左を弱くする。
   * ゲインは setBalancePid の Kp/Ki。false のときは従来のヨー PI。 */
  void setEncoderStraightBalance(bool enabled);

  static constexpr uint32_t EncoderStepMs      = 5U;
  /* エンコーダ未到達時の安全打ち切り [ms]（距離制御には使わない） */
  static constexpr uint32_t EncoderRunMaxMs    = 5000U;

  /* encoder_deg>0 なら、開始時点からの左右エンコーダ平均 [deg] で終了して stop。
   * 0 以下は指令だけ出して戻る（従来どおり）。ヨー補正は掛けたまま。 */
  void run(int left_power, int right_power, int32_t encoder_deg = 0);
  void stop();
  /* 左右モータのエンコーダ [deg] で直進／ピボットを終了する。 */
  void runFor(int left_power, int right_power, int32_t target_deg);
  /* 指定時間 [ms] 走行して台形減速停止（後退など時間制御向け）。
   * decel_ms: 終端でパワーを線形に落とす時間。省略時は DecelMs。
   * step_ms: 減速の更新間隔。大きいほど 1 段の減少幅が大きい。 */
  void runForMs(int left_power, int right_power, uint32_t duration_ms,
                uint32_t decel_ms = DecelMs, uint32_t step_ms = DecelStepMs,
                bool coast_end = false);
  /* いまの直進パワーから線形に落として停止。停止中・ピボット中は halt のみ。
   * coast_end: true なら終端は set_power(0) ではなく惰性停止（尾が上がりにくい）。 */
  void decelToStop(uint32_t decel_ms = DecelMs, uint32_t step_ms = DecelStepMs,
                   bool coast_end = false);
  /* パワー指令を止め、モータは惰性（coast）。stop() の set_power(0) は使わない。 */
  void coast();

  /* 絶対目標角までその場旋回する（両輪逆符号ピボット）。
   * 目標は 90° 刻みなど呼び出し側の固定値。経過角は積まない。
   * 左右エンコーダの回転量差でパワーを補正し、|ΔL|≈|ΔR| の完全なその場旋回にする。
   * rem <= slow_approach_deg なら slow_power に落とす（行き過ぎ防止）。
   * stop_early_deg: 目標までの残りがこの角以下で止める（慣性の行き過ぎ用）。
   *   負なら目標を通り過ぎてから止める。
   * prefer_sign: 0=最短側、+1=CW 強制、-1=CCW 強制。
   *   計画が L90/R90 なのに最短が逆向きになるのを防ぐときに使う。 */
  void turnToHeading(float target_heading,
                     float tolerance_deg = 4.0f,
                     int left_power = 40,
                     int right_power = 40,
                     uint32_t max_ms = 3000U,
                     float slow_approach_deg = 30.0f,
                     int slow_power = 33,
                     float stop_early_deg = 0.0f,
                     bool coast_end = false,
                     int prefer_sign = 0);

  /* 左右エンコーダ回転量の平均 [deg] でその場旋回を終了する（IMU は見ない）。
   * 車体 90° はトレッド≒車輪径なら約 180。左右 enc 同調 P は turnToHeading と同じ。 */
  void turnByEncoder(int32_t target_deg, bool left,
                     int left_power = 40, int right_power = 40,
                     uint32_t max_ms = 3000U,
                     int32_t slow_approach_deg = 60,
                     int slow_power = 33,
                     int32_t stop_early_deg = 0,
                     bool coast_end = false);

  void pivotCW(int power)  { run( power, -power); }
  void pivotCCW(int power) { run(-power,  power); }
  void pivotTowardDiff(int power, float diff_deg) {
    if      (diff_deg > 0.0f) pivotCW(power);
    else if (diff_deg < 0.0f) pivotCCW(power);
    else                      stop();
  }

  void tickCorrection();

  void resetEncoderCount();
  /* 左右エンコーダの生カウント [deg]（符号つき）。回転センサを直接読むので、
   * set_power で走らせている最中（LT 中など）でも値が進む。
   * 呼び出し側で開始値を控えて差分を取れば、reset なしで距離を測れる。 */
  int32_t leftEncoderCount() const;
  int32_t rightEncoderCount() const;
  /* 開始時に控えた生カウントからの走行距離 [deg]（左右の絶対値平均）。
   * 走行を止めずに測れるので、LT 中や連続直進の途中でも使える。
   * 車輪 56mm なら 360deg ≒ 176mm（約 0.49mm/deg）。 */
  int32_t traveledDegFrom(int32_t left_base, int32_t right_base) const;
  int32_t straightEncoderDeg(int left_power, int right_power) const;
  bool straightEncoderReached(int left_power, int right_power,
                              int32_t target_deg) const;
  /* ヨー補正つきエンコーダ直進の開始（brake→reset、周期補正停止）。 */
  void beginBalancedEncoderRun();
  /* ヨー補正つきエンコーダ直進の終了（brake→停止状態同期）。 */
  void endBalancedEncoderRun();
  /* ヨー補正つきでエンコーダ到達まで走行する。 */
  void runBalancedEncoderFor(int left_power, int right_power,
                             int32_t target_deg, uint32_t step_ms);

private:
  static constexpr uint32_t TurnStepMs = 1U;
  /* その場旋回: 左右エンコーダ回転量を揃える P 補正
   * （機械差で片輪だけ進むと並進成分が出るため） */
  static constexpr float TurnEncBalanceKp       = 2.5f;
  static constexpr int   TurnEncBalanceMaxCorr  = 30;
  static constexpr int   TurnEncBalanceMinPower = 12;
  static constexpr int32_t TurnEncBalanceDeadbandDeg = 0;
  /* 左輪が強い機械差。旋回中の左パワーから常に引く（大きいほど左を弱める）。
   * P は差が出てから切るので、左が先に進む癖はこれで先に殺す。 */
  static constexpr int TurnLeftPowerTrim = 7;

  static int clampMotorPower(int power);
  static int clampInt(int value, int min_value, int max_value);
  static float clampFloat(float value, float min_value, float max_value);
  static bool samePowerRatio(int left, int right, int prev_left, int prev_right);
  static bool isSameStraightPower(int left_power, int right_power);
  static int32_t absEncoderDeg(int32_t count);
  static bool wheelEncoderReached(int32_t count, int power, int32_t target_deg);
  /* 旋回左右マグニチュード。enc_diff = |ΔL|-|ΔR|。左 trim 込み */
  static void turnWheelMags(int base, int32_t enc_diff, int *l_mag, int *r_mag);

  void setPowerRaw(int left_power, int right_power);
  void applyWithCorrection(int left_power, int right_power);
  void resetCorrection();
  void syncStoppedState();
  bool encoderReached(int left_power, int right_power, int32_t target_deg);

  pup_motor_t *left_ = nullptr;
  pup_motor_t *right_ = nullptr;

  /* resetEncoderCount() 時点の生カウント。pup_motor_reset_count() は
   * サーボ更新ループが止まっていると効かないので、ソフト側でも 0 点を持つ。 */
  int32_t enc_zero_left_  = 0;
  int32_t enc_zero_right_ = 0;

  float   target_heading_   = 0.0f;
  float   integral_         = 0.0f;
  int     last_cmd_left_    = 0;
  int     last_cmd_right_   = 0;
  bool    cmd_initialized_  = false;
  bool    correction_enabled_ = true;
  bool    heading_correction_ = true;
  bool    encoder_straight_   = false;
  bool    correction_debug_   = false;
  int32_t enc_bal_left_       = 0;
  int32_t enc_bal_right_      = 0;
  bool    manual_target_heading_ = false;
  uint8_t correction_debug_div_ = 0;
  float   kp_               = DefaultBalanceKp;
  float   ki_               = DefaultBalanceKi;
  float   i_max_            = BalanceIntegralMax;
  float   deadband_deg_     = 0.0f;
};

#endif  // TESTAPP_STRAIGHT_H_
