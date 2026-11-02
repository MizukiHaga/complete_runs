#ifndef NANSHO3_KYOKU_PATH_H
#define NANSHO3_KYOKU_PATH_H

namespace kyoku {

constexpr float Pi = 3.14159265358979323846f;

struct Vec {
  float x;
  float y;
};

/* 車軸中央の姿勢。位置:mm / 向き:rad、左旋回(CCW)が正。 */
struct Pose {
  Vec p;
  float yaw;
};

/* ゲート開口中央と、その開口を横切る方向。 */
struct Gate {
  Vec center;
  float normal;
};

/* 経路モジュール単体でも使える既定値。
 * 実機の調整値はSettings.hにまとめ、config()で全項目を設定する。 */
struct Config {
  float wheelDiameter = 56.0f;
  float track         = 120.0f;
  float halfWidth     = 80.0f;
  float front         = 100.0f;
  float rear          = 80.0f;
  float height        = 200.0f;

  float gateWidth  = 240.0f;
  float gateHeight = 245.0f;
  float footRadius = 25.0f;
  float margin     = 30.0f;
  float minX = -250.0f;
  float maxX = 1250.0f;
  float minY = -250.0f;
  float maxY = 1250.0f;

  float sample    = 8.0f;
  float minRadius = 100.0f;
  int laps        = 6;
  /* 比較する曲線形状の数。1なら基本候補だけ、3なら広い曲線も比較。 */
  int curveStyles = 3;
  bool optimizeTime = true;
  bool fastPlan = true;  // Hub向け: 直接接続を優先し、最初の成立候補で終了

  float maxSpeed            = 220.0f;
  float gateSpeed           = 130.0f;
  float wheelSpeed          = 320.0f;
  float acceleration        = 180.0f;
  float lateralAcceleration = 180.0f;

  float lookaheadMin     = 45.0f;
  float lookaheadMax     = 90.0f;
  float lookaheadTime    = 0.25f;
  float maxTrackingError = 25.0f;
};

/* 経路上のサンプル。sは始点からの距離、speedは速度計画の上限。 */
struct Sample {
  Vec p;
  float yaw;
  float curvature;
  float s;
  float speed;
};

struct Command {
  float left;
  float right;
  float curvature;
  bool done;
  bool valid;
};

/* 停止後のC→数字で示す接触予測の対象。 */
enum class MotionRisk : unsigned char {
  Safe = 0,
  Boundary = 1,
  RedGate = 2,
  BlueGate = 3,
  YellowGate = 4,
  Invalid = 5,
};

class Path {
public:
  static constexpr int Capacity = 4096;

  /* 赤→青→黄を指定周回数通る全経路を生成する。不成立ならfalse。 */
  bool plan(const Config &config, Pose start, const Gate gates[3], Pose finish,
             bool (*keep_planning)(void *) = nullptr, void *context = nullptr);
  bool planningCancelled() const { return planning_cancelled_; }

  /* 車体の向きと余裕幅を含めて、脚・コース境界との接触を判定する。 */
  bool safe(Pose pose, float extra_margin = 0.0f) const;
  MotionRisk safety(Pose pose, float extra_margin = 0.0f) const;
  MotionRisk predictMotion(Pose pose, float left_speed, float right_speed,
                           float step_sec, int step_count, float extra_margin) const;

  /* 現在姿勢から左右速度を計算。progressは周回をまたがず単調に進める。 */
  Command follow(Pose pose, float speed, int &progress) const;

  const Sample &operator[](int index) const { return points_[index]; }
  int size() const { return count_; }
  float length() const { return length_; }
  float estimatedTime() const { return estimated_time_; }

private:
  /* 経路探索の候補数・計算量上限。通常の走行調整はSettings.hを使う。 */
  static constexpr int   GateCount              = 3;
  static constexpr int   DirectionPatternCount  = 8;
  static constexpr int   MaxCurveSampleCount    = 1200;
  static constexpr int   DetourGridHalfCount    = 2;
  static constexpr int   DetourHeadingCount     = 8;
  static constexpr float DetourGridStepMm       = 200.0f;
  static constexpr float GoalToleranceMm        = 10.0f;

  /* 曲線を評価し、emit=trueのときだけサンプル列へ追加する。 */
  bool curve(Pose start, Pose finish, float handle_ratio,
              bool emit, float *cost = nullptr);
  /* 同一姿勢間の探索結果を再利用する。失敗した区間も記憶する。 */
  struct Connection {
    Pose start;
    Pose finish;
    Pose middle;
    float scale;
    float ratio;
    bool detour;
    bool valid;
  };
  static constexpr int ConnectionCapacity = 64;
  bool connect(Pose start, Pose finish);
  bool findConnection(Pose start, Pose finish, Connection &result);
  bool continuePlanning();
  bool planFast(Pose start, Pose finish);
  float directionCost(Pose start, Pose finish, int mask) const;
  bool line(Pose start, Pose finish, int allowed_gate);
  bool route(Pose start, Pose finish, int direction_mask);
  bool append(Pose pose, float curvature, float speed_limit);
  void speeds();
  float speedLimit(float curvature, float limit) const;
  float travelTime() const;

  /* allowed_gate以外のゲートを横切る場合はtrue。-1は全て禁止。 */
  bool crosses(Vec start, Vec finish, int allowed_gate) const;

  Config config_;
  Gate gates_[GateCount];
  Vec gate_normals_[GateCount];
  Vec gate_feet_[GateCount][2];
  Connection connections_[ConnectionCapacity];
  int connection_count_ = 0;
  bool (*keep_planning_)(void *) = nullptr;
  void *planning_context_ = nullptr;
  bool planning_cancelled_ = false;
  bool allow_detours_ = true;
  Sample points_[Capacity];
  int count_ = 0;
  float length_ = 0.0f;
  float handle_scale_ = 1.0f;
  float estimated_time_ = 0.0f;
};

float wrap(float angle);
float distance(Vec start, Vec finish);
Pose odometry(Pose previous, float left_mm, float right_mm, float yaw);

} // namespace kyoku

#endif // NANSHO3_KYOKU_PATH_H
