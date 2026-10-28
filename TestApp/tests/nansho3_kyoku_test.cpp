#include "Path.h"
#include "Settings.h"
#include "SpeedControl.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
static kyoku::Path path;
/* 理想的な車輪速度モデルで全周回を走行し、ゲート通過順も独立に検証する。 */
static void simulate(kyoku::Config config, kyoku::Pose start,
                      const kyoku::Gate *gates, kyoku::Pose finish, bool mirror,
                      bool perturb = false)
{
  assert(path.plan(config, start, gates, finish));
  assert(path.size() > 10);
  for (int index = 0; index < path.size(); ++index) {
    assert(path.safe({path[index].p, path[index].yaw}));
    assert(std::isfinite(path[index].speed));
    if (index > 0 && index < path.size() - 1) {
      assert(path[index].speed > 0);
    }
  }

  kyoku::Pose pose = start;
  if (perturb) {
    pose.p.y += mirror ? -10.0f : 10.0f;
    pose.yaw += mirror ? -0.02f : 0.02f;
  }
  int progress = 0;
  int crossed = 0;
  bool done = false;
  float speed = 0;
  float last_left = 0;
  float last_right = 0;
  for (int step = 0; step < 30000; ++step) {
    const auto command = path.follow(pose, speed, progress);
    if (!command.valid) {
      std::fprintf(stderr, "invalid step=%d progress=%d xy=%f,%f\n",
                    step, progress, pose.p.x, pose.p.y);
      std::abort();
    }
    if (command.done) {
      done = true;
      break;
    }
    assert(command.left > 0 && command.right > 0);
    assert(command.left <= config.wheelSpeed + 0.01f);
    assert(command.right <= config.wheelSpeed + 0.01f);
    const float change = config.acceleration * 0.01f;
    const float left = last_left + fmaxf(-change, fminf(change, command.left - last_left));
    const float right = last_right + fmaxf(-change, fminf(change, command.right - last_right));
    const auto risk = path.predictMotion(pose, left, right,
        kyoku_settings::PredictionStepMs / 1000.0f,
        kyoku_settings::PredictionStepCount, kyoku_settings::PredictionExtraMarginMm);
    if (risk != kyoku::MotionRisk::Safe) {
      std::fprintf(stderr, "risk=%d step=%d progress=%d xy=%f,%f\n",
                    static_cast<int>(risk), step, progress, pose.p.x, pose.p.y);
    }
    assert(risk == kyoku::MotionRisk::Safe);
    last_left = left;
    last_right = right;
    speed = (left + right) / 2;
    const auto previous = pose;
    pose = kyoku::odometry(pose, left * 0.01f, right * 0.01f,
                            pose.yaw + (right - left) / config.track * 0.01f);
    assert(path.safe(pose));
    for (int gate_index = 0; gate_index < 3; ++gate_index) {
      const auto &gate = gates[gate_index];
      const float nx = cosf(gate.normal);
      const float ny = sinf(gate.normal);
      const float a = (previous.p.x - gate.center.x) * nx +
                       (previous.p.y - gate.center.y) * ny;
      const float b = (pose.p.x - gate.center.x) * nx +
                       (pose.p.y - gate.center.y) * ny;
      if (!((a <= 0 && b > 0) || (a >= 0 && b < 0))) {
        continue;
      }
      const float ratio = a / (a - b);
      const float crossing_x = previous.p.x + (pose.p.x - previous.p.x) * ratio;
      const float crossing_y = previous.p.y + (pose.p.y - previous.p.y) * ratio;
      const float lateral = fabsf(-(crossing_x - gate.center.x) * ny +
                                   (crossing_y - gate.center.y) * nx);
      if (lateral < config.gateWidth / 2) {
        assert(gate_index == crossed % 3);
        ++crossed;
      }
    }
  }
  assert(done);
  assert(crossed == 3 * config.laps);
  assert(kyoku::distance(pose.p, finish.p) < 10);
  std::printf("PASS %s%s: %d laps, %d samples, %.0f mm, %d ordered crossings\n",
              mirror ? "mirrored" : "normal", perturb ? " + offset" : "",
              config.laps, path.size(), path.length(), crossed);
}

static void testTimeSelection()
{
  kyoku::Gate gates[3];
  kyoku_settings::gates(gates);
  auto config = kyoku_settings::config();
  config.fastPlan = false;
  config.curveStyles = 1;
  assert(path.plan(config, kyoku_settings::Start, gates,
                    kyoku_settings::finishPose()));
  const float basic_time = path.estimatedTime();

  config.curveStyles = 3;
  assert(path.plan(config, kyoku_settings::Start, gates,
                    kyoku_settings::finishPose()));
  const float selected_time = path.estimatedTime();
  assert(selected_time <= basic_time + 0.001f);
  assert(selected_time > path.length() / config.maxSpeed);

  float integrated_time = 0.0f;
  for (int index = 1; index < path.size(); ++index) {
    const auto &previous = path[index - 1];
    const auto &current = path[index];
    const float step = current.s - previous.s;
    integrated_time += 2 * step / (previous.speed + current.speed);
    assert(fabsf(current.speed * current.speed - previous.speed * previous.speed)
           <= 2 * config.acceleration * step + 0.1f);
    assert(fabsf(current.curvature) <= 1 / config.minRadius + 0.00001f);
  }
  assert(fabsf(integrated_time - selected_time) < 0.001f);

  /* 距離だけなら急な曲線を選ぶ配置。僅かに長くても速い経路を採用する。 */
  config.laps = 1;
  gates[0].center.y = 100.0f;
  gates[2].center.x = 300.0f;
  config.optimizeTime = false;
  assert(path.plan(config, kyoku_settings::Start, gates,
                    kyoku_settings::finishPose()));
  const float distance_time = path.estimatedTime();
  const float distance_length = path.length();
  config.optimizeTime = true;
  assert(path.plan(config, kyoku_settings::Start, gates,
                    kyoku_settings::finishPose()));
  assert(path.estimatedTime() < distance_time - 0.05f);
  assert(path.length() > distance_length);
  std::printf("PASS time selection: distance priority %.3f s / %.3f mm, "
              "time priority %.3f s / %.3f mm\n", distance_time, distance_length,
              path.estimatedTime(), path.length());
}

static void testGridGeometry()
{
  using namespace kyoku_settings;

  const auto start = gridPoint(GridStart);
  assert(start.x == 0 && start.y == 0);
  for (int index = 0; index < 3; ++index) {
    const auto left = gridGate(GridGates[index]);
    const auto right = gridGate(GridGates[index], true);
    assert(fabsf(left.center.x - right.center.x) < 0.001f);
    assert(fabsf(left.center.y + right.center.y) < 0.001f);
    assert(fabsf(kyoku::wrap(left.normal + right.normal)) < 0.001f);
  }
  assert(!std::isfinite(gridGate({{1, 1}, {1, 1}}).normal));
  assert(!std::isfinite(gridGate({{1, 1}, {2, 2}}).normal));
  const auto left_config = config();
  const auto right_config = config(true);
  assert(right_config.minY == -left_config.maxY);
  assert(right_config.maxY == -left_config.minY);
  std::puts("PASS grid conversion, invalid gate endpoints and L/R reflection");
}

static void testWheelPower()
{
  float integral = 0;
  assert(kyoku::wheelPower(53, 0, integral, 0.01f) >= kyoku_settings::MinRunPower);
  assert(kyoku::wheelPower(1, 0, integral, 0.01f) == 35);
  for (float target : {1.0f, 20.0f, 53.0f, 130.0f, 220.0f}) {
    for (float measured : {0.0f, 50.0f, 200.0f, 1000.0f}) {
      float wheel_integral = 0;
      const int output = kyoku::wheelPower(target, measured, wheel_integral, 0.01f);
      assert(output == 0 || (output >= 35 && output <= kyoku_settings::MaxPower));
    }
  }
  assert(kyoku::wheelPower(0, 0, integral, 0.01f) == 0 && integral == 0);
  for (int index = 0; index < 500; ++index) {
    assert(kyoku::wheelPower(1000, 0, integral, 0.01f) == kyoku_settings::MaxPower);
  }
  assert(integral == 0);
  assert(kyoku::wheelPower(50, 1000, integral, 0.01f) == 0);
  std::puts("PASS minimum power, overspeed coasting, zero output and anti-windup");
}

static bool planningBudget(void *context)
{
  int &remaining = *static_cast<int *>(context);
  return --remaining > 0;
}

static void testPlanningCancellation()
{
  kyoku::Gate gates[3];
  kyoku_settings::gates(gates);
  auto config = kyoku_settings::config();
  config.fastPlan = false;
  for (int budget : {1, 100, 2000}) {
    int remaining = budget;
    assert(!path.plan(config, kyoku_settings::Start, gates,
                       kyoku_settings::finishPose(), planningBudget, &remaining));
    assert(remaining == 0);
    assert(path.planningCancelled());
    assert(path.size() == 0 && path.length() == 0 && path.estimatedTime() == 0);
    int progress = 0;
    assert(!path.follow(kyoku_settings::Start, 0, progress).valid);
  }
  /* 前回の中断結果・接続キャッシュを次回の計画へ持ち越さない。 */
  assert(path.plan(config, kyoku_settings::Start, gates,
                    kyoku_settings::finishPose()));
  assert(!path.planningCancelled());
  std::puts("PASS planning cancellation, invalidated partial route and restart");
}

static bool countPlanningWork(void *context)
{
  ++*static_cast<int *>(context);
  return true;
}

static void testFastPlanning()
{
  kyoku::Gate gates[3];
  kyoku_settings::gates(gates);
  auto config = kyoku_settings::config();
  config.fastPlan = false;
  int full_work = 0;
  assert(path.plan(config, kyoku_settings::Start, gates,
                    kyoku_settings::finishPose(), countPlanningWork, &full_work));
  config.fastPlan = true;
  int fast_work = 0;
  assert(path.plan(config, kyoku_settings::Start, gates,
                    kyoku_settings::finishPose(), countPlanningWork, &fast_work));
  assert(fast_work < full_work / 2);
  assert(path.estimatedTime() > 0);
  std::printf("PASS planning work: full %d checks, fast %d checks\n", full_work, fast_work);
  // 軽量モードでも中断でき、途中経路を追従へ渡さない。
  int remaining = 1;
  assert(!path.plan(config, kyoku_settings::Start, gates,
                     kyoku_settings::finishPose(), planningBudget, &remaining));
  assert(path.planningCancelled() && path.size() == 0);
}

static void testCollisionDetails()
{
  kyoku::Gate gates[3];
  kyoku_settings::gates(gates);
  const auto config = kyoku_settings::config();
  assert(path.plan(config, kyoku_settings::Start, gates, kyoku_settings::finishPose()));
  assert(path.safety(kyoku_settings::Start) == kyoku::MotionRisk::Safe);
  assert(path.safety({{config.minX - 1, 0}, 0}) == kyoku::MotionRisk::Boundary);
  for (int index = 0; index < 3; ++index) {
    const auto gate = gates[index];
    const float offset = config.gateWidth / 2 + config.footRadius;
    const kyoku::Pose foot = {{gate.center.x - sinf(gate.normal) * offset,
                               gate.center.y + cosf(gate.normal) * offset}, gate.normal};
    assert(path.safety(foot) == static_cast<kyoku::MotionRisk>(index + 2));
    assert(path.predictMotion(foot, 0, 0, 0.02f, 10, 0) ==
           static_cast<kyoku::MotionRisk>(index + 2));
  }
  assert(path.safety({{NAN, 0}, 0}) == kyoku::MotionRisk::Invalid);
  assert(path.predictMotion(kyoku_settings::Start, NAN, 0, 0.02f, 10, 0) ==
         kyoku::MotionRisk::Invalid);
  std::puts("PASS collision prediction and boundary/red/blue/yellow/invalid details");
}

int main(int argc, char **argv)
{
  testGridGeometry();
  testWheelPower();
  testTimeSelection();
  testPlanningCancellation();
  testFastPlanning();
  testCollisionDetails();

  kyoku::Gate configured_gates[3];
  kyoku_settings::gates(configured_gates);
  const auto configured_finish = kyoku_settings::finishPose();
  auto config = kyoku_settings::config();
  simulate(config, kyoku_settings::Start, configured_gates, configured_finish, false);
  if (argc == 2) {
    FILE *file = std::fopen(argv[1], "w");
    assert(file);
    std::fprintf(file, "x,y,yaw,curvature,s,speed\n");
    for (int index = 0; index < path.size(); ++index) {
      const auto &point = path[index];
      std::fprintf(file, "%f,%f,%f,%f,%f,%f\n", point.p.x, point.p.y,
                    point.yaw, point.curvature, point.s, point.speed);
    }
    std::fclose(file);
  }

  kyoku::Gate mirrored_gates[3];
  kyoku_settings::gates(mirrored_gates, true);
  simulate(kyoku_settings::config(true), kyoku_settings::Start,
            mirrored_gates, kyoku_settings::finishPose(true), true);
  simulate(config, kyoku_settings::Start, configured_gates, configured_finish,
            false, true);
  int progress = 0;
  assert(!path.follow({{1500, 1500}, 0}, 0, progress).valid);
  assert(!path.follow(kyoku_settings::Start, NAN, progress).valid);

  config.height = config.gateHeight;
  assert(!path.plan(config, kyoku_settings::Start, configured_gates, configured_finish));
  assert(path.size() == 0 && path.estimatedTime() == 0);
  config = kyoku_settings::config();
  config.halfWidth = 130;
  assert(!path.plan(config, kyoku_settings::Start, configured_gates, configured_finish));
  config = kyoku_settings::config();
  config.sample = 0;
  assert(!path.plan(config, kyoku_settings::Start, configured_gates, configured_finish));
  config = kyoku_settings::config();
  config.curveStyles = 4;
  assert(!path.plan(config, kyoku_settings::Start, configured_gates, configured_finish));
  auto pose = kyoku::odometry({{0, 0}, 0}, 100, 100, 0);
  assert(fabsf(pose.p.x - 100) < 0.001f);
  pose = kyoku::odometry({{0, 0}, kyoku::Pi - 0.01f}, 10, 10, -kyoku::Pi + 0.01f);
  assert(pose.p.x < -9.99f);
  std::puts("PASS invalid geometry, off-path stop, signed odometry and yaw wrap");
}
