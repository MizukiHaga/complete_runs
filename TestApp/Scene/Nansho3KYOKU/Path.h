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

class Path {
public:
  static constexpr int Capacity = 4096;

  /* 赤→青→黄を指定周回数通る全経路を生成する。不成立ならfalse。 */
  bool plan(const Config &config, Pose start, const Gate gates[3], Pose finish);

  /* 車体の向きと余裕幅を含めて、脚・コース境界との接触を判定する。 */
  bool safe(Pose pose, float extra_margin = 0.0f) const;

  /* 現在姿勢から左右速度を計算。progressは周回をまたがず単調に進める。 */
  Command follow(Pose pose, float speed, int &progress) const;

  const Sample &operator[](int index) const { return points_[index]; }
  int size() const { return count_; }
  float length() const { return length_; }

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
  bool connect(Pose start, Pose finish);
  bool line(Pose start, Pose finish, int allowed_gate);
  bool route(Pose start, Pose finish, int direction_mask);
  bool append(Pose pose, float curvature, float speed_limit);
  void speeds();

  /* allowed_gate以外のゲートを横切る場合はtrue。-1は全て禁止。 */
  bool crosses(Vec start, Vec finish, int allowed_gate) const;

  Config config_;
  Gate gates_[GateCount];
  Sample points_[Capacity];
  int count_ = 0;
  float length_ = 0.0f;
  bool store_points_ = false;
};

float wrap(float angle);
float distance(Vec start, Vec finish);
Pose odometry(Pose previous, float left_mm, float right_mm, float yaw);

} // namespace kyoku

#endif // NANSHO3_KYOKU_PATH_H
