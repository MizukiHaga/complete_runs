#include "Path.h"

#include <math.h>

namespace kyoku {
namespace {

Vec add(Vec first, Vec second)
{
  return {first.x + second.x, first.y + second.y};
}

Vec subtract(Vec first, Vec second)
{
  return {first.x - second.x, first.y - second.y};
}

Vec scale(Vec value, float factor)
{
  return {value.x * factor, value.y * factor};
}

float dot(Vec first, Vec second)
{
  return first.x * second.x + first.y * second.y;
}

Vec unit(float angle)
{
  return {cosf(angle), sinf(angle)};
}

float clamp(float value, float minimum, float maximum)
{
  return fmaxf(minimum, fminf(value, maximum));
}

} // namespace

float wrap(float angle)
{
  while (angle > Pi) {
    angle -= 2 * Pi;
  }
  while (angle < -Pi) {
    angle += 2 * Pi;
  }
  return angle;
}

float distance(Vec start, Vec finish)
{
  const Vec delta = subtract(start, finish);
  return sqrtf(dot(delta, delta));
}

Pose odometry(Pose previous, float left_mm, float right_mm, float yaw)
{
  const float yaw_delta = wrap(yaw - previous.yaw);
  const float arc_factor = fabsf(yaw_delta) < 0.0001f
                             ? 1.0f : sinf(yaw_delta / 2) / (yaw_delta / 2);
  const float center_distance = (left_mm + right_mm) * 0.5f * arc_factor;
  const Vec movement = scale(unit(previous.yaw + yaw_delta / 2), center_distance);
  return {add(previous.p, movement), yaw};
}

bool Path::safe(Pose pose, float extra_margin) const
{
  return safety(pose, extra_margin) == MotionRisk::Safe;
}

MotionRisk Path::safety(Pose pose, float extra_margin) const
{
  if (!isfinite(pose.p.x) || !isfinite(pose.p.y) || !isfinite(pose.yaw)) {
    return MotionRisk::Invalid;
  }

  const float margin = config_.margin + extra_margin;
  const Vec forward = unit(pose.yaw);
  const Vec side = {-forward.y, forward.x};

  /* 車体の四隅が、余裕幅を含めてコース内にあることを確認する。 */
  for (int corner_index = 0; corner_index < 4; ++corner_index) {
    const float forward_offset = (corner_index & 1)
                                  ? config_.front + margin : -config_.rear - margin;
    const float side_offset = (corner_index & 2)
                               ? config_.halfWidth + margin : -config_.halfWidth - margin;
    const Vec corner = add(pose.p, add(scale(forward, forward_offset),
                                       scale(side, side_offset)));
    if (corner.x < config_.minX || corner.x > config_.maxX ||
        corner.y < config_.minY || corner.y > config_.maxY) {
      return MotionRisk::Boundary;
    }
  }

  /* 脚を円、車体を向き付き矩形として接触を調べる。
   * 円には柱だけでなく、継手と足元の張り出しも含める。 */
  for (int gate_index = 0; gate_index < GateCount; ++gate_index) {
    for (int foot_index = 0; foot_index < 2; ++foot_index) {
      const Vec delta = subtract(gate_feet_[gate_index][foot_index], pose.p);
      const float local_x = dot(delta, forward);
      const float local_y = dot(delta, side);
      const float delta_x = local_x - clamp(local_x, -config_.rear - margin,
                                                      config_.front + margin);
      const float delta_y = local_y - clamp(local_y, -config_.halfWidth - margin,
                                                      config_.halfWidth + margin);
      if (delta_x * delta_x + delta_y * delta_y <=
          config_.footRadius * config_.footRadius) {
        return static_cast<MotionRisk>(gate_index + 2);
      }
    }
  }
  return MotionRisk::Safe;
}

MotionRisk Path::predictMotion(Pose pose, float left_speed, float right_speed,
                               float step_sec, int step_count, float extra_margin) const
{
  if (!isfinite(left_speed) || !isfinite(right_speed) ||
      !isfinite(step_sec) || step_sec <= 0 || step_count < 1) {
    return MotionRisk::Invalid;
  }
  const float yaw_rate = (right_speed - left_speed) / config_.track;
  for (int step = 0; step < step_count; ++step) {
    pose = odometry(pose, left_speed * step_sec, right_speed * step_sec,
                    pose.yaw + yaw_rate * step_sec);
    const MotionRisk risk = safety(pose, extra_margin);
    if (risk != MotionRisk::Safe) {
      return risk;
    }
  }
  return MotionRisk::Safe;
}

bool Path::crosses(Vec start, Vec finish, int allowed_gate) const
{
  for (int gate_index = 0; gate_index < GateCount; ++gate_index) {
    if (gate_index == allowed_gate) {
      continue;
    }

    const Gate &gate = gates_[gate_index];
    const Vec normal = gate_normals_[gate_index];
    const Vec tangent = {-normal.y, normal.x};
    const float start_side = dot(subtract(start, gate.center), normal);
    const float finish_side = dot(subtract(finish, gate.center), normal);
    const bool changes_side = (start_side <= 0 && finish_side > 0) ||
                               (start_side >= 0 && finish_side < 0);
    if (!changes_side) {
      continue;
    }

    const float ratio = start_side / (start_side - finish_side);
    const Vec crossing = add(start, scale(subtract(finish, start), ratio));
    const float lateral = fabsf(dot(subtract(crossing, gate.center), tangent));
    const float blocked_width = config_.gateWidth / 2 +
                                 config_.halfWidth + config_.margin;
    if (lateral < blocked_width) {
      return true;
    }
  }
  return false;
}

bool Path::append(Pose pose, float curvature, float speed_limit)
{
  if (count_ > 0) {
    const int previous_index = count_ - 1;
    const float step_distance = distance(points_[previous_index].p, pose.p);
    if (step_distance < 0.001f) {
      /* 接続点は重複保存せず、両側の速度制約を厳しい方へ統合する。 */
      Sample &previous = points_[previous_index];
      previous.speed = fminf(previous.speed, speedLimit(curvature, speed_limit));
      return true;
    }
    length_ += step_distance;
  }
  if (count_ >= Capacity) {
    return false;
  }

  const Sample sample = {pose.p, pose.yaw, curvature, length_,
                          speedLimit(curvature, speed_limit)};
  points_[count_] = sample;
  ++count_;
  return true;
}

bool Path::curve(Pose start, Pose finish, float handle_ratio,
                  bool emit, float *cost)
{
  if (!continuePlanning()) {
    return false;
  }
  const float chord_length = distance(start.p, finish.p);
  if (chord_length < 1.0f) {
    return fabsf(wrap(start.yaw - finish.yaw)) < 0.01f;
  }

  /* 3次ベジェ。始点・終点の接線を車体の向きに合わせる。 */
  const Vec control1 = add(start.p, scale(unit(start.yaw), chord_length * handle_ratio));
  const Vec control2 = subtract(finish.p, scale(unit(finish.yaw), chord_length * handle_ratio));
  const float length_bound = distance(start.p, control1) +
                             distance(control1, control2) + distance(control2, finish.p);
  if (!isfinite(length_bound) || length_bound > MaxCurveSampleCount * config_.sample) {
    return false;
  }

  const int step_count = static_cast<int>(ceilf(length_bound / config_.sample));
  float curve_cost = 0.0f;
  float previous_speed = config_.maxSpeed;
  Vec previous_point = start.p;
  for (int step = 0; step <= step_count; ++step) {
    if (step % 32 == 0 && !continuePlanning()) {
      return false;
    }
    const float t = static_cast<float>(step) / step_count;
    const float u = 1.0f - t;
    const Vec point = add(
        add(scale(start.p, u * u * u), scale(control1, 3 * u * u * t)),
        add(scale(control2, 3 * u * t * t), scale(finish.p, t * t * t)));
    const Vec velocity = scale(add(
        add(scale(subtract(control1, start.p), u * u),
            scale(subtract(control2, control1), 2 * u * t)),
        scale(subtract(finish.p, control2), t * t)), 3);
    const Vec acceleration = scale(add(
        scale(add(subtract(control2, scale(control1, 2)), start.p), u),
        scale(add(subtract(finish.p, scale(control2, 2)), control1), t)), 6);

    const float velocity_squared = dot(velocity, velocity);
    if (velocity_squared < 0.01f) {
      return false;
    }
    const float curvature = (velocity.x * acceleration.y - velocity.y * acceleration.x) /
                             (velocity_squared * sqrtf(velocity_squared));
    const Pose pose = {point, atan2f(velocity.y, velocity.x)};

    /* サンプル間の移動と、車体回転による端の移動も余裕に加える。 */
    const float body_radius = fmaxf(config_.front, config_.rear) + config_.halfWidth;
    const float sweep_margin = config_.sample * (1 + body_radius / config_.minRadius);
    if (fabsf(curvature) > 1 / config_.minRadius ||
        !safe(pose, sweep_margin) || crosses(previous_point, point, -1)) {
      return false;
    }
    if (emit && !append(pose, curvature, config_.maxSpeed)) {
      return false;
    }
    if (cost != nullptr) {
      const float speed = speedLimit(curvature, config_.maxSpeed);
      const float step_distance = distance(previous_point, point);
      curve_cost += config_.optimizeTime
                      ? 2 * step_distance / (previous_speed + speed) : step_distance;
      previous_speed = speed;
    }
    previous_point = point;
  }
  if (cost != nullptr) {
    *cost = curve_cost;
  }
  return true;
}

bool Path::continuePlanning()
{
  if (!planning_cancelled_ && keep_planning_ != nullptr) {
    planning_cancelled_ = !keep_planning_(planning_context_);
  }
  return !planning_cancelled_;
}

bool Path::connect(Pose start, Pose finish)
{
  if (!continuePlanning()) {
    return false;
  }
  Connection result = {};
  bool cached = false;
  for (int index = 0; index < connection_count_; ++index) {
    const Connection &candidate = connections_[index];
    if (candidate.scale == handle_scale_ &&
        candidate.start.p.x == start.p.x && candidate.start.p.y == start.p.y &&
        candidate.start.yaw == start.yaw &&
        candidate.finish.p.x == finish.p.x && candidate.finish.p.y == finish.p.y &&
        candidate.finish.yaw == finish.yaw) {
      result = candidate;
      cached = true;
      break;
    }
  }
  if (!cached) {
    result.start = start;
    result.finish = finish;
    result.scale = handle_scale_;
    result.valid = findConnection(start, finish, result);
    if (planning_cancelled_) {
      return false;
    }
    if (connection_count_ < ConnectionCapacity) {
      connections_[connection_count_++] = result;
    }
  }
  if (!result.valid) {
    return false;
  }
  if (result.detour) {
    return curve(start, result.middle, result.ratio, true) &&
           curve(result.middle, finish, result.ratio, true);
  }
  return curve(start, finish, result.ratio, true);
}

bool Path::findConnection(Pose start, Pose finish, Connection &result)
{
  const float handle_ratios[] = {0.3f, 0.5f, 0.8f, 1.2f};
  float best_cost = 1e30f;
  float best_handle_ratio = 0.0f;
  Pose best_middle = {};
  bool found_detour = false;

  /* まず1本のベジェで接続し、曲率による減速も含めて比較する。 */
  for (float base_ratio : handle_ratios) {
    const float handle_ratio = base_ratio * handle_scale_;
    float cost = 0.0f;
    if (curve(start, finish, handle_ratio, false, config_.fastPlan ? nullptr : &cost) && cost < best_cost) {
      best_cost = cost;
      best_handle_ratio = handle_ratio;
      if (config_.fastPlan) {
        break; // 軽量モードでは局所最適化のために残りの曲線を評価しない
      }
    }
  }
  if (best_handle_ratio > 0.0f) {
    result.ratio = best_handle_ratio;
    result.detour = false;
    return true;
  }

  /* 直接接続できない場合だけ、中間姿勢を1つ加えた候補を探索する。
   * 有限候補による迂回探索であり、大域的最短経路の保証はしない。 */
  if (!allow_detours_) {
    return false;
  }
  const int grid_half = config_.fastPlan ? 1 : DetourGridHalfCount;
  const int heading_count = config_.fastPlan ? 4 : DetourHeadingCount;
  const Vec midpoint = scale(add(start.p, finish.p), 0.5f);
  for (int grid_x = -grid_half; grid_x <= grid_half; ++grid_x) {
    for (int grid_y = -grid_half; grid_y <= grid_half; ++grid_y) {
      const Vec offset = {grid_x * DetourGridStepMm, grid_y * DetourGridStepMm};
      const Vec middle_point = add(midpoint, offset);
      for (int heading = 0; heading < heading_count; ++heading) {
        if (!continuePlanning()) {
          return false;
        }
        const Pose middle = {middle_point, heading * 2 * Pi / heading_count};
        if (!safe(middle)) {
          continue;
        }
        const float distance_bound = distance(start.p, middle_point) +
                                      distance(middle_point, finish.p);
        const float lower_bound = config_.optimizeTime
                                    ? distance_bound / config_.maxSpeed : distance_bound;
        if (lower_bound >= best_cost) {
          continue;
        }
        for (float base_ratio : handle_ratios) {
          const float handle_ratio = base_ratio * handle_scale_;
          float first_cost = 0.0f;
          float second_cost = 0.0f;
          if (curve(start, middle, handle_ratio, false, config_.fastPlan ? nullptr : &first_cost) &&
              curve(middle, finish, handle_ratio, false, config_.fastPlan ? nullptr : &second_cost) &&
              first_cost + second_cost < best_cost) {
            best_cost = first_cost + second_cost;
            best_handle_ratio = handle_ratio;
            best_middle = middle;
            found_detour = true;
            if (config_.fastPlan) {
              result.ratio = best_handle_ratio;
              result.middle = best_middle;
              result.detour = true;
              return true;
            }
          }
        }
      }
    }
  }
  result.ratio = best_handle_ratio;
  result.middle = best_middle;
  result.detour = found_detour;
  return found_detour;
}

bool Path::line(Pose start, Pose finish, int allowed_gate)
{
  const int step_count = static_cast<int>(ceilf(distance(start.p, finish.p) / config_.sample));
  Vec previous_point = start.p;
  for (int step = 0; step <= step_count; ++step) {
    if (step % 32 == 0 && !continuePlanning()) {
      return false;
    }
    const float ratio = step_count > 0 ? static_cast<float>(step) / step_count : 0.0f;
    const Pose pose = {add(start.p, scale(subtract(finish.p, start.p), ratio)), start.yaw};
    if (!safe(pose, config_.sample) || crosses(previous_point, pose.p, allowed_gate) ||
        !append(pose, 0.0f, config_.gateSpeed)) {
      return false;
    }
    previous_point = pose.p;
  }
  return true;
}

bool Path::route(Pose current, Pose finish, int direction_mask)
{
  count_ = 0;
  length_ = 0.0f;
  if (!append(current, 0.0f, config_.maxSpeed)) {
    return false;
  }

  /* 後端がゲートを抜けるまで直線を保ち、先読みが早く曲がるのを防ぐ。 */
  const float clearance = fmaxf(config_.front, config_.rear) + config_.margin +
                           config_.lookaheadMax + config_.footRadius;
  for (int lap = 0; lap < config_.laps; ++lap) {
    for (int gate_index = 0; gate_index < GateCount; ++gate_index) {
      const Gate &gate = gates_[gate_index];
      const bool reverse_direction = (direction_mask & (1 << gate_index)) != 0;
      const float yaw = wrap(gate.normal + (reverse_direction ? Pi : 0.0f));
      const Vec normal = unit(yaw);
      const Pose entry = {subtract(gate.center, scale(normal, clearance)), yaw};
      const Pose exit = {add(gate.center, scale(normal, clearance)), yaw};
      if (!connect(current, entry) || !line(entry, exit, gate_index)) {
        return false;
      }
      current = exit;
    }
  }
  return connect(current, finish);
}

float Path::speedLimit(float curvature, float limit) const
{
  const float magnitude = fabsf(curvature);
  float speed = fminf(limit, config_.wheelSpeed /
                             (1 + magnitude * config_.track / 2));
  if (magnitude > 0.00001f) {
    speed = fminf(speed, sqrtf(config_.lateralAcceleration / magnitude));
  }
  return speed;
}

float Path::travelTime() const
{
  float time = 0.0f;
  for (int index = 1; index < count_; ++index) {
    const float speed_sum = points_[index - 1].speed + points_[index].speed;
    if (speed_sum <= 0.0f) {
      return 1e30f;
    }
    /* 区間内を等加速度とみなす。加減速制約反映後の速度を使用。 */
    time += 2 * (points_[index].s - points_[index - 1].s) / speed_sum;
  }
  return time;
}

void Path::speeds()
{
  /* 終点から逆向きに、停止までに必要な減速距離を確保する。 */
  points_[count_ - 1].speed = 0.0f;
  for (int index = count_ - 2; index >= 0; --index) {
    const float step_distance = points_[index + 1].s - points_[index].s;
    const float next_speed = points_[index + 1].speed;
    const float speed_limit = sqrtf(next_speed * next_speed +
                                    2 * config_.acceleration * step_distance);
    points_[index].speed = fminf(points_[index].speed, speed_limit);
  }

  /* 始点から順向きに、停止からの加速上限を反映する。 */
  points_[0].speed = 0.0f;
  for (int index = 1; index < count_; ++index) {
    const float step_distance = points_[index].s - points_[index - 1].s;
    const float previous_speed = points_[index - 1].speed;
    const float speed_limit = sqrtf(previous_speed * previous_speed +
                                    2 * config_.acceleration * step_distance);
    points_[index].speed = fminf(points_[index].speed, speed_limit);
  }
}

/* ゲート通過軸から区間の直線距離・向きの変化を見積もる。
 * 衝突判定を伴わない軽い計算で、試す通過方向の順番だけを決める。 */
float Path::directionCost(Pose current, Pose finish, int mask) const
{
  const float clearance = fmaxf(config_.front, config_.rear) + config_.margin +
                           config_.lookaheadMax + config_.footRadius;
  float cost = 0.0f;
  for (int lap = 0; lap < config_.laps; ++lap) {
    for (int index = 0; index < GateCount; ++index) {
      const float yaw = wrap(gates_[index].normal + ((mask & (1 << index)) ? Pi : 0));
      const Vec offset = scale(unit(yaw), clearance);
      const Vec entry = subtract(gates_[index].center, offset);
      cost += distance(current.p, entry) + 2 * clearance +
              config_.minRadius * fabsf(wrap(yaw - current.yaw));
      current = {add(gates_[index].center, offset), yaw};
    }
  }
  return cost + distance(current.p, finish.p) +
         config_.minRadius * fabsf(wrap(finish.yaw - current.yaw));
}

bool Path::planFast(Pose start, Pose finish)
{
  int order[DirectionPatternCount];
  float costs[DirectionPatternCount];
  for (int pattern = 0; pattern < DirectionPatternCount; ++pattern) {
    order[pattern] = pattern;
    costs[pattern] = directionCost(start, finish, pattern);
  }
  for (int index = 1; index < DirectionPatternCount; ++index) {
    const int pattern = order[index];
    int position = index;
    while (position > 0 && costs[pattern] < costs[order[position - 1]]) {
      order[position] = order[position - 1];
      --position;
    }
    order[position] = pattern;
  }

  handle_scale_ = 1.0f;
  /* 先に直接接続だけで全方向を調べる。失敗する方向の迂回探索で待たせない。
   * 直接つながらない場合だけ、候補数を絞った迂回を試す。 */
  for (int pass = 0; pass < 2; ++pass) {
    allow_detours_ = pass != 0;
    connection_count_ = 0; // 直接接続の失敗キャッシュを迂回探索へ持ち越さない
    for (int index = 0; index < DirectionPatternCount; ++index) {
      const bool valid = route(start, finish, order[index]);
      if (planning_cancelled_) {
        count_ = 0;
        length_ = 0.0f;
        return false;
      }
      if (valid && count_ > 1) {
        speeds();
        estimated_time_ = travelTime();
        return true; // 全候補の比較・採用候補の再生成は行わない
      }
    }
  }
  count_ = 0;
  length_ = 0.0f;
  return false;
}

bool Path::plan(const Config &config, Pose start, const Gate gates[3], Pose finish,
                  bool (*keep_planning)(void *), void *context)
{
  keep_planning_ = keep_planning;
  planning_context_ = context;
  planning_cancelled_ = false;
  connection_count_ = 0;
  allow_detours_ = true;
  config_ = config;
  count_ = 0;
  length_ = 0.0f;
  estimated_time_ = 0.0f;

  /* 不正な設定で探索や距離の除算へ進まないようにする。 */
  const float values[] = {
    config.wheelDiameter, config.track, config.halfWidth, config.front,
    config.rear, config.height, config.gateWidth, config.gateHeight,
    config.footRadius, config.margin, config.sample, config.minRadius,
    config.minX, config.maxX, config.minY, config.maxY, config.maxSpeed,
    config.gateSpeed, config.wheelSpeed, config.acceleration,
    config.lateralAcceleration, config.lookaheadMin, config.lookaheadMax,
    config.lookaheadTime, config.maxTrackingError,
  };
  for (float value : values) {
    if (!isfinite(value)) {
      return false;
    }
  }
  const bool invalid_geometry =
      config.minX >= config.maxX || config.minY >= config.maxY ||
      config.wheelDiameter <= 0 || config.track <= 0 || config.halfWidth <= 0 ||
      config.front <= 0 || config.rear <= 0 || config.height <= 0 ||
      config.gateWidth <= 0 || config.gateHeight <= 0 || config.footRadius <= 0 ||
      config.height >= config.gateHeight;
  const bool invalid_path_settings =
      config.sample <= 0 || config.minRadius <= config.track / 2 ||
      config.laps < 1 || config.laps > 10 ||
      config.curveStyles < 1 || config.curveStyles > 3;
  const bool invalid_speed_settings =
      config.acceleration <= 0 || config.lateralAcceleration <= 0 ||
      config.maxSpeed <= 0 || config.gateSpeed <= 0 || config.wheelSpeed <= 0;
  const bool invalid_tracking_settings =
      config.lookaheadMin <= 0 || config.lookaheadMax < config.lookaheadMin ||
      config.lookaheadTime < 0 || config.maxTrackingError <= 0 ||
      config.margin < config.maxTrackingError;
  if (invalid_geometry || invalid_path_settings ||
      invalid_speed_settings || invalid_tracking_settings) {
    return false;
  }

  for (int gate_index = 0; gate_index < GateCount; ++gate_index) {
    const Gate &gate = gates[gate_index];
    if (!isfinite(gate.center.x) || !isfinite(gate.center.y) || !isfinite(gate.normal)) {
      return false;
    }
    gates_[gate_index] = gate;
    gate_normals_[gate_index] = unit(gate.normal);
    const Vec tangent = unit(gate.normal + Pi / 2);
    const float offset = config_.gateWidth / 2 + config_.footRadius;
    gate_feet_[gate_index][0] = subtract(gate.center, scale(tangent, offset));
    gate_feet_[gate_index][1] = add(gate.center, scale(tangent, offset));
  }
  if (!safe(start) || !safe(finish)) {
    return false;
  }

  if (config_.fastPlan) {
    return planFast(start, finish);
  }

  /* 通過方向8組×曲線形状を評価する。停止は始点・終点のみ。
   * 候補ごとに同じバッファを使い、選んだ候補だけ最後に再生成する。 */
  const float handle_scales[] = {1.0f, 0.75f, 1.4f};
  float best_cost = 1e30f;
  int best_pattern = -1;
  int best_style = -1;
  for (int style = 0; style < config_.curveStyles; ++style) {
    handle_scale_ = handle_scales[style];
    for (int pattern = 0; pattern < DirectionPatternCount; ++pattern) {
      const bool valid_route = route(start, finish, pattern);
      if (planning_cancelled_) {
        count_ = 0;
        length_ = 0.0f;
        return false;
      }
      if (!valid_route || count_ < 2) {
        continue;
      }
      speeds();
      const float cost = config_.optimizeTime ? travelTime() : length_;
      if (cost < best_cost) {
        best_cost = cost;
        best_pattern = pattern;
        best_style = style;
      }
    }
  }
  count_ = 0;
  length_ = 0.0f;
  if (best_pattern < 0) {
    return false;
  }

  handle_scale_ = handle_scales[best_style];
  if (!route(start, finish, best_pattern)) {
    count_ = 0;
    length_ = 0.0f;
    return false;
  }
  speeds();
  estimated_time_ = travelTime();
  return count_ > 1;
}

Command Path::follow(Pose pose, float speed, int &progress) const
{
  Command command = {0.0f, 0.0f, 0.0f, false, false};
  if (count_ < 2 || progress < 0 || progress >= count_ ||
      !isfinite(speed) || !safe(pose)) {
    return command;
  }

  /* 近傍だけを単調に探索し、交差点で次周の経路へ飛ばないようにする。 */
  int nearest_index = progress;
  float nearest_distance = distance(pose.p, points_[nearest_index].p);
  for (int index = progress + 1; index < count_; ++index) {
    if (points_[index].s - points_[progress].s > config_.lookaheadMax * 2) {
      break;
    }
    const float candidate_distance = distance(pose.p, points_[index].p);
    if (candidate_distance < nearest_distance) {
      nearest_index = index;
      nearest_distance = candidate_distance;
    }
  }
  if (nearest_distance > config_.maxTrackingError) {
    return command;
  }
  progress = nearest_index;
  if (nearest_index >= count_ - 3 &&
      distance(pose.p, points_[count_ - 1].p) < GoalToleranceMm) {
    command.done = true;
    command.valid = true;
    return command;
  }

  /* 車体から先読み距離以上離れた、前方の経路点を目標にする。 */
  const float lookahead = clamp(fabsf(speed) * config_.lookaheadTime,
                                config_.lookaheadMin, config_.lookaheadMax);
  int target_index = nearest_index;
  while (target_index + 1 < count_ &&
         distance(pose.p, points_[target_index].p) < lookahead) {
    ++target_index;
  }

  const Vec delta = subtract(points_[target_index].p, pose.p);
  const float distance_squared = dot(delta, delta);
  if (distance_squared < 1.0f) {
    return command;
  }
  const float lateral = -sinf(pose.yaw) * delta.x + cosf(pose.yaw) * delta.y;
  const float forward = cosf(pose.yaw) * delta.x + sinf(pose.yaw) * delta.y;
  if (forward <= 0.0f) {
    return command;
  }

  /* Pure Pursuit: 曲率 = 2×目標点の車体横方向位置 / 距離^2。 */
  const float curvature = 2 * lateral / distance_squared;
  if (fabsf(curvature) > 2 / config_.track) {
    return command; // 片輪が逆転する指令は出さない
  }

  /* 始点の速度は0なので、次のサンプルの速度で走り出す。 */
  const int speed_index = nearest_index + 1 < count_ ? nearest_index + 1 : nearest_index;
  float target_speed = points_[speed_index].speed;
  if (speed_index == count_ - 1) {
    /* 終点サンプルは0。手前で停止指令にせず、終点までの残距離で減速する。 */
    const float remaining = distance(pose.p, points_[count_ - 1].p);
    target_speed = fminf(points_[nearest_index].speed,
                         sqrtf(2 * config_.acceleration * remaining));
  }
  const float wheel_limit = config_.wheelSpeed / (1 + fabsf(curvature) * config_.track / 2);
  target_speed = fminf(target_speed, wheel_limit);
  if (fabsf(curvature) > 0.00001f) {
    target_speed = fminf(target_speed, sqrtf(config_.lateralAcceleration / fabsf(curvature)));
  }

  command.left  = target_speed * (1 - curvature * config_.track / 2);
  command.right = target_speed * (1 + curvature * config_.track / 2);
  command.curvature = curvature;
  command.valid = true;
  return command;
}

} // namespace kyoku
