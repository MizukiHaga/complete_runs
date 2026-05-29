/*
 * Straight モジュール ヘッダ
 *  左右モータをまとめて操作する直進・蛇行の共通モジュール。
 *  pup_motor_* の C API は Straight クラスの中だけで使い、
 *  外からは run/stop/runFor/runSerpentine などのメソッドで操作する。
 *
 *  使い方:
 *    Straight gMotors;
 *    gMotors.configure(PBIO_PORT_ID_B, PBIO_PORT_ID_A);   // 左, 右
 *    gMotors.run(70, 65);              // 走る
 *    gMotors.stop();                   // 止める
 *    gMotors.runFor(70, 65, 2000);     // 2 秒走って減速停止
 *    gMotors.runSerpentine(20, 20, 15, 300, 0, poll);  // 蛇行
 */

#ifndef TESTAPP_STRAIGHT_H_
#define TESTAPP_STRAIGHT_H_

#include <pbio/port.h>
#include <stdbool.h>
#include <stdint.h>

/* PUP の C API は C リンクで提供されているので extern "C" で囲む */
extern "C" {
#include <spike/pup/motor.h>
}

class Straight {
public:
  /* 蛇行中に毎ステップ呼ばれるコールバックの型。
   * 引数なし・bool 戻り値で、true を返すと runSerpentine がループを抜けて停止する。
   * メンバ関数を直接渡せないので、シーン側で static 関数を作り、
   * その中でグローバル参照経由でセンサにアクセスする運用にしている。 */
  using PollFn = bool (*)(void);

  /* === 既定値 / 定数 ====================================== */
  /* 直進時のデフォルト左右パワー。機械差を吸収するため左右で別値にできる。 */
  static constexpr int DefaultLeftPower = 70;
  static constexpr int DefaultRightPower = 65;
  /* straight_task が runFor で直進する時間 [ms] */
  static constexpr uint32_t DefaultTimeMs = 2000U;
  /* runFor の最後で線形減速にかける時間 [ms] */
  static constexpr uint32_t DecelMs = 800U;
  /* 減速中のパワー更新間隔 [ms] (小さいほど滑らかだが負荷が増える) */
  static constexpr uint32_t DecelStepMs = 40U;
  /* dly_tsk の上限 TMAX_RELTIM (4,000,000 ms) を超えないようにするためのクランプ */
  static constexpr uint32_t MaxDurationMs = 4000000U;
  /* 蛇行のステア量・周期・既定時間 */
  static constexpr int SerpentineSteer = 15;
  static constexpr uint32_t SerpentinePhaseMs = 300U;
  static constexpr uint32_t SerpentineTimeMs = 2000U;
  /* runSerpentine の total_ms にこれを渡すと時間制限なしで走り続ける (= 0) */
  static constexpr uint32_t SerpentineForever = 0U;
  /* フェーズ反転時 (2 回目以降) に挫む直進セグメントの長さ [ms]。
   * 0 にすると以前の挙動 (反転直後にすぐ反対側へステア) になる。 */
  static constexpr uint32_t SerpentineStraightMs = 900U;

  /* === API ================================================ */
  /* 初期化: 左右モータデバイスを取得し、回転方向を設定する。
   * 第3引数 true でエンコーダをリセット。 */
  void configure(pbio_port_id_t left_motor_port, pbio_port_id_t right_motor_port);

  /* 左右両方の PUP モータが取れているか。 */
  bool isConfigured() const { return left_ != nullptr && right_ != nullptr; }

  /* 左右モータに指定パワーをセットする。すぐ戻る。
   * 正: 前進方向 / 負: 後退方向 / 0: 惰性停止 (ブレーキではない) */
  void run(int left_power, int right_power);

  /* 左右モータのパワーを 0 にする (惰性停止)。 */
  void stop();

  /* 指定パワーで duration_ms だけ走り、最後は線形減速で 0 まで落として停止する (ブロッキング)。
   * duration_ms == 0 のときは停止のみ。
   * MaxDurationMs を超える指定は MaxDurationMs にクランプ。 */
  void runFor(int left_power, int right_power, uint32_t duration_ms);

  /* 蛇行走行 (ブロッキング)。
   *   left_base, right_base : 基準パワー
   *   steer                  : 左右のパワー差 (大きいほど大きく蛇行)
   *   phase_ms               : ステア方向の基準周期 [ms]
   *                            初回フェーズは phase_ms、2 回目以降は
   *                            phase_ms*2 を使う (直進開始位置から中心線を
   *                            またいで両側に均等に振れるようにするため)
   *                            フェーズ反転時 (2 回目以降) には
   *                            SerpentineStraightMs だけ base/base で直進する
   *   total_ms               : 全体の走行時間 [ms]
   *                            SerpentineForever (0) で時間制限なし
   *   poll                   : 10ms ごとに呼ばれる関数。
   *                            true を返すと即座に蛇行を停止する。
   *                            nullptr のときは時間だけで判定する。 */
  void runSerpentine(int left_base, int right_base, int steer, uint32_t phase_ms,
                     uint32_t total_ms, PollFn poll);

private:
  /* モータパワーを [-100, 100] に丸める。範囲外の値が来てもクラッシュしないため。 */
  static int clampMotorPower(int power);
  /* 蛇行ループの 1 ステップの長さ [ms]。poll の最小反応間隔と同じ。 */
  static constexpr uint32_t SerpentineStepMs = 10U;

  /* 取得した左右モータデバイス。configure() で初期化、以降の関数で使い回す。 */
  pup_motor_t *left_ = nullptr;
  pup_motor_t *right_ = nullptr;
};

/* === 後方互換マクロ ======================================
 * 旧 C スタイルのコードから参照されている名前を、クラスの定数や型に橋渡しする。
 * 新規コードでは Straight::DefaultLeftPower のように直接書く方がよい。 */
#define STRAIGHT_LEFT_POWER        Straight::DefaultLeftPower
#define STRAIGHT_RIGHT_POWER       Straight::DefaultRightPower
#define STRAIGHT_TIME              Straight::DefaultTimeMs
#define STRAIGHT_DECEL_MS          Straight::DecelMs
#define STRAIGHT_DECEL_STEP_MS     Straight::DecelStepMs
#define STRAIGHT_MAX_DURATION_MS   Straight::MaxDurationMs
#define STRAIGHT_SERPENTINE_STEER      Straight::SerpentineSteer
#define STRAIGHT_SERPENTINE_PHASE_MS   Straight::SerpentinePhaseMs
#define STRAIGHT_SERPENTINE_TIME       Straight::SerpentineTimeMs
#define STRAIGHT_SERPENTINE_FOREVER    Straight::SerpentineForever
typedef Straight::PollFn straight_serpentine_poll_fn;

#endif  // TESTAPP_STRAIGHT_H_
