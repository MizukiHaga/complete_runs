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
   *  ⓪ 開始時: ジャイロ初期化 → InitialBlackTurn(180°) → その後 LT
   *  ① 低速 LT → 本線 LT で円の終端まで移動 (エンコーダ距離 BackToEndDeg)
   *  ② 円の方向へ旋回 (右 90°)、旋回後の向きを 0° にリセット、
   *     指定時間 (ForwardBeforeScanMs) 前進
   *  ③ 1回目走査 (LR共通で Right): 検知 → 灰色まで → finishAfterDetect(Right=180°) → ⑦
   *  ④ 検知なしで規定回数完了 → 円の反対を読み取る動き
   *       (readOppositeSide: reset → 直進(時間) → 旋回 → reset → 直進(時間) → 旋回 → reset → 灰色まで直進 → reset)
   *  ⑤ 2回目走査 (Left): 検知 → 灰色まで → finishAfterDetect(Left・0°基準) → ⑦
   *  ⑥ 検知なしで規定回数完了 → 黒線を読む動き → ⑦
   *  ⑦ 黒線読み後: 指定方向に 90° 旋回 → 低速LT → 青線までライントレース
   *  ※ 振り角の符号のみ ETROBO_COURSE_L で反転。ScanSide の先右後左は LR 共通。
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
   *   front_heading: 走査の正面 [deg]（1 回目=円向き reset 後 0°、2 回目=反対読み reset 後 0°）
   *   走査中はジャイロ reset しない
   *   返り値:
   *     SonicDetected → out_heading に検知時ヨー角
   *     BlackDetected → 前進中に黒線エッジを踏んだ
   *     Completed     → max_cycles を検知なしで完了
   *
   *   黒線の判定は前進フェーズでのみ行う。
   *   その場ピボット旋回中はカラーセンサが弧を描いて元の黒線を
   *   横切り直してしまい誤終了するため、旋回中は黒を見ない。 */
  ScanResult wigwagScan(float amp_deg, int max_cycles, float *out_heading,
                        ScanSide scan_side, float front_heading = 0.0f);

  /* 円の反対を読み取る動き: reset → 直進 → 旋回 → reset → 直進 → 旋回 → reset → 灰色まで直進 → reset。 */
  ScanResult readOppositeSide();

  /* 黒線を読む動き: target_heading を向いてから黒線エッジを踏むまで前進。 */
  void driveUntilBlackLine(float target_heading = 0.0f);

  /* 灰色ラインを踏むまで直進して停止。
   * gray_ignore_ms: 開始直後この時間 [ms] は灰色判定を無視 (0 で無効)。
   * max_ms: 打ち切り [ms] (0 で無制限)。 */
  void driveUntilGrayLine(uint32_t gray_ignore_ms = DetectGrayIgnoreMs,
                          int left_power = LeftPower,
                          int right_power = RightPower,
                          uint32_t max_ms = 0U);

  /* 検知後の終了動作 (走査方向で分岐)。灰色直進後に待機・バックしてから分岐。
   *   heading_offset_deg: 2 回目走査の正面 (反対読み reset 後 0°)。Left の角度分岐に使用。
   *   Right: 円側 0° 基準 180° → 黒線まで直進
   *   Left:  浅い → 反対側 0° → 黒線 / 広い → 反対側 ±90° → 直進 → 反対側 0° → 黒線 */
  void finishAfterDetect(ScanSide side, float heading_offset_deg = 0.0f);

  /* 現在の向きのまま黒線エッジを踏むまで前進して停止。 */
  void driveForwardUntilBlackLine();

  /* 旋回しながら超音波だけ監視 (黒線は見ない)。
   *   tolerance_deg: 目標角到達とみなす許容幅 [deg] */
  ScanResult rotateToHeadingWithScan(float target_heading, float *out_heading,
                                     ScanSide scan_side, float tolerance_deg);

  /* 直進しながら超音波 + 黒線エッジを監視。 */
  /* 直進中スキャン: duration_ms だけ power で前進しつつ超音波/黒を監視。
   * ヨー保持は driveBalancedStraight と同じ手順 (目標角は呼び出し側で指定)。 */
  ScanResult driveForwardWithScan(uint32_t duration_ms, int power,
                                  float *out_heading, float target_heading);

  /* 円向き後直進 / wigwag 直進の共通: LT・周期補正を止め、PI ヨー保持で直進。 */
  void beginBalancedStraight(int left_power, int right_power,
                             float target_heading);
  void endBalancedStraight();

  /* 現在のヨー角を target_heading に合わせるピボット旋回 (検知判定なし)。
   *   tolerance_deg: 目標角到達とみなす許容幅 [deg] (省略時 TurnToleranceDeg)。
   *   ジャイロ reset は行わない。必要なら呼び出し側で gGyro.reset() する。 */
  void turnToHeading(float target_heading,
                     float tolerance_deg = TurnToleranceDeg);

  /* 左右モーター個別パワーでピボット旋回 (符号は旋回方向に合わせて付与)。
   *   left_power / right_power: 正のパワー値 (CW 時 L=+, R=− / CCW 時 L=−, R=+)。 */
  void turnToHeading(float target_heading, float tolerance_deg,
                     int left_power, int right_power);

  /* その場で 360° ピボット旋回しながら超音波を読み続ける (即時停止式)。 */
  bool scan360(float *out_heading);

  /* ③ 1回目 wig走査 → ④ 反対読み → ⑤ 2回目 wig走査。検知時 true。 */
  bool runWigwagScanPhases();

  /* 開始時: ジャイロ初期化のみ（旋回なし。低速 LT から開始）。 */
   void runInitialForwardTrace();

  /* 最終黒線読み後: 指定方向に 90° 旋回 → LT → SKYBLUE で停止。 */
  void runAfterFinalBlackLine();
  void turnAfterFinalBlackLine();

  /* ① のエンコーダ距離ログ（調整用）。 */
  static void logBackTraceEncoder(const char *phase, int32_t traveled_deg,
                                  int32_t target_deg, uint32_t steps);

  /* === ⓪ 開始時 (ライントレース) パラメータ */
  static constexpr float    InitialTraceKp      = 0.30f;
  static constexpr float    InitialTraceKi      = 0.01f;
  static constexpr float    InitialTraceKd      = 0.8f;
  static constexpr float    InitialTraceOutMax  = 100.0f;
  static constexpr float    InitialTraceScale   = 1.0f;  /* HCR 後も自前 PID を維持 */
  static constexpr float    InitialBlackTurnDeg = 180.0f;

  /* === ⑦ 最終黒線読み後 (旋回 → LT → SKYBLUE) パラメータ */
  /* 黒線読み後の旋回。L と R を鏡像にせず、それぞれ独立に設定する。
   * 旋回角は現在角からの相対量 [deg]（CW 正 = 右旋回, 負 = 左旋回）。
   * パワーは左右モータへそのまま入る値（正 = 前進, 負 = 後退）。
   * 試合前にこの 6 個だけ調整する。 */
  /* R コース（ETROBO_COURSE_L が 1） */
  static constexpr float    FinalBlueTraceTurnDegR        =  90.0f;
  static constexpr int      FinalBlueTraceTurnLeftPowerR  =  60;
  static constexpr int      FinalBlueTraceTurnRightPowerR = -40;
  /* L コース（ETROBO_COURSE_L が 0） */
  static constexpr float    FinalBlueTraceTurnDegL        = -90.0f;
  static constexpr int      FinalBlueTraceTurnLeftPowerL  = -40;
  static constexpr int      FinalBlueTraceTurnRightPowerL =  60;

  /* 実際に使う値。上のどちらかが選ばれるだけなので、ここは触らない。 */
  static constexpr float    FinalBlueTraceTurnDeg =
      ETROBO_COURSE_L ? FinalBlueTraceTurnDegR : FinalBlueTraceTurnDegL;
  static constexpr int      FinalBlueTraceTurnLeftPower =
      ETROBO_COURSE_L ? FinalBlueTraceTurnLeftPowerR
                      : FinalBlueTraceTurnLeftPowerL;
  static constexpr int      FinalBlueTraceTurnRightPower =
      ETROBO_COURSE_L ? FinalBlueTraceTurnRightPowerR
                      : FinalBlueTraceTurnRightPowerL;
  static constexpr float    FinalBlueTraceTurnToleranceDeg  = 1.0f;
  static constexpr uint32_t FinalBlueTraceTurnMaxMs         = 8000U;
  /* 旋回直後の低速 LT（本線 LT の前に線へ乗せる）。 */
  static constexpr float    FinalSlowTracePower  = 35.0f;
  static constexpr uint32_t FinalSlowTraceMs     = 400U;
  static constexpr float    FinalSlowTraceKp     = 0.30f;
  static constexpr float    FinalSlowTraceKi     = 0.01f;
  static constexpr float    FinalSlowTraceKd     = 0.8f;
  static constexpr float    FinalSlowTraceOutMax = 100.0f;
  static constexpr float    FinalSlowTraceScale  = 1.0f;
  /* 本線 LT（SKYBLUE/BLUE 検知で終了）。 */
  static constexpr float    FinalBlueTracePower             = 80.0f;
  static constexpr float    FinalBlueTraceKp                = 0.65f;
  static constexpr float    FinalBlueTraceKi                = 0.0001f;
  static constexpr float    FinalBlueTraceKd                = 0.8f;
  static constexpr float    FinalBlueTraceOutMax            = 100.0f;
  static constexpr float    FinalBlueTraceScale             = 1.0f;
  /* 追従エッジ: L=左エッジ / R=右エッジ */
  static constexpr bool     FinalBlueTraceEdgeFollow = (ETROBO_COURSE_L == 1);
  /* 旋回直後の誤検知防止。低速LT中も含めた経過時間で数える [ms] */
  static constexpr uint32_t FinalBlueTraceColorIgnoreMs     = 200U;
  static constexpr uint32_t FinalBlueTraceMaxMs             = 15000U;

  /* === ① バック・ライントレース (円の終端まで) パラメータ =========
   * 終了条件はエンコーダ距離。距離は左右エンコーダ [deg] の平均で、
   * ⓪ の旋回直後（低速 LT の開始時）からの累計。
   * 車輪 56mm なら 1 回転 360deg ≒ 176mm（約 0.49mm/deg）。 */
  /* 旋回直後の低速 LT (本線 LT の前に線へ乗せる)。 */
  static constexpr int      BackSlowTracePower = 35;
  /* 低速 LT を終える累計距離 [deg]。 */
  static constexpr int32_t  BackSlowTraceDeg   = 188;
//   static constexpr int32_t  BackSlowTraceDeg   = 190;
  /* ① 全体を終える累計距離 [deg]（低速 LT の分を含む）。 */
  static constexpr int32_t  BackToEndDeg       = 1600;
  /* 調整用: エンコーダ距離をシリアルに出す。調整後は false に戻す。 */
  static constexpr bool     BackTraceEncoderLog      = false;
  static constexpr uint32_t BackTraceEncoderLogSteps = 200U;
  /* バック・ライントレースの左右パワー (負値で後退)。 */
  static constexpr int      BackTracePower     = 80;
  static constexpr float    BackTraceKp        = 0.6f;
  static constexpr float    BackTraceKi        = 0.0005f;
  static constexpr float    BackTraceKd        = 0.8f;
  static constexpr float    BackTraceOutMax    = 100.0f;
  static constexpr float    BackTraceScale     = 1.0f;
  /* バック LT 中の減速: DecelDeg 進む間に Power → EndPower へ線形に下げる。 */
  static constexpr int      BackTraceEndPower  = 30;
  static constexpr int32_t  BackTraceDecelDeg  = 2550;

  /* === 検知共通パラメータ ==================================== */
  /* 走査検知: 超音波距離がこの値以下なら障害物ありとみなす [mm]。 */
  static constexpr int32_t  DetectMaxMm        = 280;

  /* === 旋回走査 (wigwag) パラメータ ========================== */
  /* 1 方向あたりの「規定回数」サイクル数。９
   * これだけ繰り返して検知が無ければ次の動作へ移る。 */
  static constexpr int      ScanCycles         = 6;
  /* 振る角度 [°] (右走査は +、左走査は -)。
   * 1サイクル = 側方へ旋回 → 正面へ戻り → 少し前進 (reset なし)。 */
  static constexpr float    WigWagAmpDeg       = 90.0f;
  /* 1 サイクル末の「少し直進」する時間 [ms] とパワー。 */
  static constexpr uint32_t WigWagForwardMs    = 250U;
  static constexpr int      WigWagForwardPower = 40;
  /* 左走査の旋回中に超音波検知したとき、停止せず旋回を継続する時間 [ms]。 */
  static constexpr uint32_t LeftScanDetectContinueMs = 0U;

  /* === 360° スキャンフェーズ用パラメータ ===================== */
  /* ピボット旋回パワー (左右逆符号で同値、その場旋回) */
  static constexpr int      ScanTurnPower          = 40;
  /* 走査旋回の許容幅 [deg] (大きいほど早めに到達扱い)。 */
  /* 側方 90° への往路旋回 */
  static constexpr float    WigWagOutToleranceDeg    = 6.0f;
  /* 正面へ戻る復路旋回 */
  static constexpr float    WigWagReturnToleranceDeg = 6.0f;
  /* スキャンを打ち切る最大経過時間 [ms] (安全用) */
  static constexpr uint32_t ScanMaxMs              = 15000U;

  /* === 目標方向への旋回パラメータ ============================ */
  static constexpr int      TurnPower          = 40;
  static constexpr float    TurnToleranceDeg   = 2.0f;
  /* この角度以内になったら低速に落とす [deg]（行き過ぎ防止） */
  static constexpr float    TurnSlowApproachDeg = 30.0f;
  static constexpr int      TurnSlowPower       = 33;
  /* turnToHeading の打ち切り時間 [ms] (安全用) */
  static constexpr uint32_t TurnMaxMs          = 8000U;

  /* === 円の反対を読み取る動き (readOppositeSide) パラメータ === */
  static constexpr int      OppositeForwardPower       = 50;
  /* 直進はエンコーダ距離で終わる。左右エンコーダ [deg] の平均で、
   * 車輪 56mm なら 360deg ≒ 176mm（約 0.49mm/deg）。 */
  static constexpr int32_t  OppositeForwardFirstDeg    = 360;  /* 1 回目直進 [deg] */
  static constexpr int32_t  OppositeForwardSecondDeg   = 300;  /* 2 回目直進 [deg] */
  static constexpr int      OppositeTurnPower          = 45;
  /* 反対読み: 各セグメント開始時に reset し、旋回はその 0° 基準の相対角。
   * 1 回目旋回=±90°、2 回目旋回=±90° (1 回目 reset 後)。 */
  static constexpr float    OppositeFirstTurnDeg         =
      ETROBO_COURSE_L ? -90.0f : 90.0f;
  static constexpr float    OppositeSecondTurnDeg        =
      ETROBO_COURSE_L ? -90.0f : 90.0f;
  static constexpr float    OppositeTurnToleranceDeg   = 4.0f;

  /* === ② 旋回後の前進 (走査開始前) パラメータ =============== */
  /* 円方向への旋回角 [deg] (app.h で L/R 切り替え)。 */
  static constexpr float    PreWigwagTurnDeg           =
      ETROBO_COURSE_L ? -90.0f : 90.0f;
  /* 旋回 + ヨー角リセットのあと、走査前に前進する時間 [ms]。 */
  static constexpr uint32_t ForwardBeforeScanMs = 200U;
  static constexpr float    PreWigwagBalanceKp          = 3.0f;
  static constexpr float    PreWigwagBalanceKi          = 0.01f;
  static constexpr float    PreWigwagBalanceIntegralMax = 50.0f;
  /* 直進ヨー角 PI 補正。false で open-loop（検証用。戻すときは true）。 */
  static constexpr bool     UseHeadingCorrection        = true;

  /* === 検知後の終了動作 (finishAfterDetect) パラメータ ========= */
  /* 超音波検知後の灰色直進: 開始直後この時間 [ms] は灰色判定を無視する。 */
  static constexpr uint32_t DetectGrayIgnoreMs    = 300U;
  /* 検知後共通: 灰色停止からバック開始までの待ち時間 [ms] (慣性で後部が上がるのを防ぐ)。 */
  static constexpr uint32_t DetectFinishBackDelayMs = 300U;
  /* 検知後共通: 灰色直進後のバック時間 [ms] / パワー。 */
  static constexpr uint32_t DetectFinishBackMs    = 700U;
  static constexpr int      DetectFinishBackPower = 50;
  /* 右走査 (円側 reset 後): 黒線方向 = 絶対 180°。 */
  static constexpr float    DetectFinishRightTurnDeg = 180.0f;
  /* 左走査 (反対読み reset 後): 黒線方向 = 正面 0°
   * (円側 0° 基準の 180° と同じ向き)。 */
  static constexpr float    DetectFinishLeftTurnDeg = 0.0f;
  /* 左走査検知後の角度分岐。|相対角| がこの値以内なら迂回ルート、
   * 外なら 0° へ旋回して黒線まで直進。 */
  static constexpr int      DetectFinishAngleSign =
      ETROBO_COURSE_L ? -1 : 1;
  static constexpr float    DetectFinishClassicMaxDeg =
      ETROBO_COURSE_L ? 27.0f : -27.0f;
  /* 左走査・迂回ルート (反対読み reset 後の絶対角)。
   * 1回目: L=-90° / R=+90° (円側 ±90° から 180° オフセット) */
  static constexpr float    DetectWideTurnFirstDeg =
      ETROBO_COURSE_L ? 90.0f : -90.0f;
  /* 2回目: 反対側 0° (= 黒線方向) */
  static constexpr float    DetectWideTurnSecondDeg = 0.0f;
  /* 迂回ルート: 1回目旋回後の直進時間 [ms]。 */
  static constexpr uint32_t DetectMidForwardMs          = 1500U;

  /* === ジャイロ reset の下駄 [deg] =========================== */
  /* Nansho1 の gGyro.reset() すべてに載せる。CW 正。
   * reset 直後の heading() が 0° ではなくこの値になるので、
   * 目標 0° の直進は車体をこの符号と逆へ向ける。
   * -2 なら車体は 2° CW（右）。左へずらしたいなら正にする。
   * L と R は鏡像にせず、それぞれ独立に設定する。 */
  static constexpr float    GyroResetOffsetDegR = 1.7f;  /* R コース */
  static constexpr float    GyroResetOffsetDegL = -1.7f;  /* L コース */
  /* 実際に使う値。ここは触らない。 */
  static constexpr float    GyroResetOffsetDeg =
      ETROBO_COURSE_L ? GyroResetOffsetDegR : GyroResetOffsetDegL;

  /* === 制御ループ周期 [ms] =================================== */
  static constexpr uint32_t StepMs             = 1U;
};

#ifdef __cplusplus
extern "C" {
#endif

void Nansho1Task(intptr_t exinf);

#ifdef __cplusplus
}
#endif

#endif  // TESTAPP_NANSHO1_H_
