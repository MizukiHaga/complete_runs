#!/usr/bin/env python3
"""実ソースの旋回方向選択と角度折り返しを抽出し、旧45度境界を検証する。"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'Common/Straight/Straight.cpp').read_text()
start = source.index('  float turn_dir =', source.index('void Straight::turnToHeading('))
end = source.index('  const float early =', start)
selection = source[start:end]
gyro = (root / 'Common/Gyro/Gyro.cpp').read_text()
start = gyro.index('float Gyro::wrapDeg180(')
end = gyro.index('\n}\n', start) + 3
wrap = gyro[start:end]
prefix = r'''
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
class Gyro { public: static float wrapDeg180(float); };
static void near(float actual, float expected) {
  assert(fabsf(actual - expected) < 0.001f);
}
'''
function = '''float plannedTurn(float from, float target, int prefer_sign) {
  const float delta = Gyro::wrapDeg180(target - from);
''' + selection + '''  return turn_dir * need;
}
'''
main = r'''
int main() {
  for (int prefer : {-1, 0, 1}) {
    near(plannedTurn(0, -44.9f, prefer), -44.9f);
    near(plannedTurn(0, -45.1f, prefer), -45.1f);
    near(plannedTurn(0, 44.9f, prefer), 44.9f);
    near(plannedTurn(0, 45.1f, prefer), 45.1f);
    for (float from : {-720.0f, -179.0f, 0.0f, 179.0f, 720.0f}) {
      for (float target : {-179.0f, -90.0f, 0.0f, 90.0f, 179.0f}) {
        const float chosen = plannedTurn(from, target, prefer);
        assert(fabsf(chosen) <= 180);
        near(Gyro::wrapDeg180(from + chosen - target), 0);
      }
    }
  }
  near(plannedTurn(179, -179, -1), 2);
  near(plannedTurn(-179, 179, 1), -2);
  for (float target : {-180.0f, 180.0f}) {
    near(plannedTurn(0, target, -1), -180);
    near(plannedTurn(0, target, 1), 180);
  }
  puts("turn_direction_test: PASS (45-degree continuity, shortest rotation and 180-degree tie)");
}
'''
with tempfile.TemporaryDirectory(prefix='turn-direction-') as temp:
    cpp = Path(temp) / 'test.cpp'
    binary = Path(temp) / 'test'
    cpp.write_text(prefix + wrap + function + main)
    subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror',
                    str(cpp), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
