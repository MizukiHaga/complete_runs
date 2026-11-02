#!/usr/bin/env python3
"""実ソースのエンコーダ減速を抽出し、距離判定・ブレーキ・安全抜けを検証する。"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'Scene/Nansho3/Nansho3.cpp').read_text()
header = (root / 'Scene/Nansho3/Nansho3.h').read_text()
header = header.replace('#include <kernel.h>', '').replace('#include "app.h"', '')
header = header.replace('private:', 'public:')
start = source.index('static float smoothstep01(')
end = source.index('\n#if ETROBO_COURSE_L', start)
helpers = source[start:end]
start = source.index('void Nansho3Scene::decelToStopByEncoder(')
end = source.index('\nvoid Nansho3Scene::driveOneCell', start)
decel = source[start:end]

harness = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
struct Motors {
  float position = 100;
  float speed = 0;
  float target = 0;
  float brake_position = 0;
  int brake_count = 0;
  int elapsed_ms = 0;
  bool stalled = false;
  bool braking = false;
  int32_t traveledDegFrom(int32_t left, int32_t right) const {
    return static_cast<int32_t>(position) - (left + right) / 2;
  }
  void run(int left, int right) {
    assert(left == right);
    assert(!braking);
    target = std::abs(left) / 90.0f;
  }
  void brake() {
    ++brake_count;
    brake_position = position;
    braking = true;
  }
} gMotors;
void dly_tsk(unsigned usec) {
  for (unsigned t = 0; t < usec / 1000; ++t) {
    ++gMotors.elapsed_ms;
    if (!gMotors.stalled) {
      gMotors.speed += gMotors.braking
                        ? -0.5f * gMotors.speed
                        : 0.1f * (gMotors.target - gMotors.speed);
      gMotors.position += gMotors.speed;
    }
  }
}
'''
main = r'''
void Nansho3Scene::logDriveControl(const char *, bool, float) {}
int main() {
  Nansho3Scene scene;
  for (float entry_speed : {0.5f, 1.0f, 2.0f}) {
    gMotors = Motors();
    gMotors.position = 455;  // 基点100、355度進んだ位置から残り150度を減速
    gMotors.speed = entry_speed;
    scene.decelToStopByEncoder(100, 100, 505, 90, 150);
    assert(gMotors.brake_count == 1);
    assert(gMotors.brake_position >= 605 && gMotors.brake_position < 607);
    assert(gMotors.position - gMotors.brake_position < 2);
    assert(gMotors.speed < 0.001f);
  }
  gMotors = Motors();
  gMotors.stalled = true;
  scene.decelToStopByEncoder(100, 100, 505, 90, 150);
  assert(gMotors.brake_count == 1);
  assert(gMotors.elapsed_ms <= int(scene.CoreStraightDecelStallMs +
                                   scene.CoreStraightBrakeSettleMs));
  assert(gMotors.position == 100);
  // すでに目標距離を超えている場合、追加の駆動を出さずブレーキに移る。
  gMotors = Motors();
  gMotors.position = 610;
  scene.decelToStopByEncoder(100, 100, 505, 90, 150);
  assert(gMotors.brake_count == 1 && gMotors.target == 0);
  std::puts("PASS distance includes deceleration, active brake, settling and stall exit");
}
'''
with tempfile.TemporaryDirectory(prefix='nansho3-brake-') as temp:
    cpp = Path(temp) / 'test.cpp'
    binary = Path(temp) / 'test'
    cpp.write_text(harness + header + helpers + decel + main)
    subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-DETROBO_COURSE_L=0', str(cpp), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
