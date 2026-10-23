#!/usr/bin/env python3
"""実ソースの経路計画・実行順を抽出し、モータ動作だけ座標シミュレーションに置換。"""
import argparse
from pathlib import Path
import subprocess
import tempfile

base = Path(__file__).resolve().parents[1] / 'Scene/Nansho3'
p = argparse.ArgumentParser()
p.add_argument('--source', type=Path, default=base / 'Nansho3.cpp')
p.add_argument('--header', type=Path, default=base / 'Nansho3.h')
args = p.parse_args()
source = args.source.read_text()
header = args.header.read_text().replace('#include <kernel.h>', '').replace('#include "app.h"', '').replace('private:', 'public:')
names = ['snapTo90', 'facingIndex', 'facingDegFromIndex', 'inGrid',
         'isYAdjacent', 'isXAdjacent', 'isOrthoAdjacent', 'nearerGateEnd',
         'clearBlocked', 'blockPoint', 'blockGates', 'blocksStep', 'bfsPlan',
         'bfsCost', 'planStartsWithFwd', 'passGateStartsWithFwd',
         'planStartsWithFwdTowardGate', 'goTo', 'passGateShortest', 'runRallyLap']
functions = []
for name in names:
    pos = source.find('Nansho3Scene::' + name + '(')
    if pos < 0:
        continue
    start = source.rfind('\n', 0, pos) + 1
    end = source.index('\n}\n', pos) + 3
    functions.append(source[start:end])
prefix = r'''
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <vector>
struct Gyro { static float wrapDeg180(float x) {
  while (x >= 180) x -= 360; while (x < -180) x += 360; return x;
}};
struct Motors { void coast() {} } gMotors;
'''
harness = r'''
static const int kDx[4] = {1,0,-1,0};
#if ETROBO_COURSE_L
static const int kDy[4] = {0,1,0,-1};
#else
static const int kDy[4] = {0,-1,0,1};
#endif
static std::vector<int> crossed;
static GateSeg gates[3];
static int errors = 0;
static int first_motion = -1;
void Nansho3Scene::beepError() const { ++errors; }
void Nansho3Scene::setNextTarget(QrPoint) {}
void Nansho3Scene::onArrivedCell() {}
void Nansho3Scene::onGatePassed() { ++gates_since_gyro_reset_; }
void Nansho3Scene::turnByAction(MoveAction a) {
  if (first_motion < 0) first_motion = 1;
  int f = facingIndex(facing_deg_);
  facing_deg_ = facingDegFromIndex((f + (a == MoveAction::L90 ? 3 : 1)) % 4);
}
void Nansho3Scene::driveOneCell(bool stop) {
  if (first_motion < 0) first_motion = 0;
  int f = facingIndex(facing_deg_), nx = x_ + kDx[f], ny = y_ + kDy[f];
  for (int i = 0; i < 3; ++i) {
    const GateSeg &g = gates[i];
    if ((x_ == g.a.x && y_ == g.a.y && nx == g.b.x && ny == g.b.y) ||
        (x_ == g.b.x && y_ == g.b.y && nx == g.a.x && ny == g.a.y)) crossed.push_back(i);
  }
  x_ = nx; y_ = ny; straight_running_ = !stop;
}
static bool runCase(GateSeg r, GateSeg b, GateSeg y, int facing, int laps) {
  Nansho3Scene s;
  s.x_ = 1; s.y_ = 1; s.facing_deg_ = s.facingDegFromIndex(facing);
  s.straight_running_ = false; s.has_next_target_ = false;
  gates[0] = r; gates[1] = b; gates[2] = y; crossed.clear(); errors = 0;
  for (int lap = 0; lap < laps; ++lap) {
    const GateSeg future[2] = {b,y};
    bool predicted_fwd = s.passGateStartsWithFwd(s.x_,s.y_,s.facingIndex(s.facing_deg_),r,nullptr,0,future,2);
    first_motion = -1;
    s.runRallyLap(r,b,y,lap+1<laps);
    if ((first_motion == 0) != predicted_fwd) return false;
  }
  if (errors || crossed.size() != static_cast<unsigned>(laps * 3)) return false;
  for (unsigned i = 0; i < crossed.size(); ++i) if (crossed[i] != int(i%3)) return false;
  return true;
}
int main() {
  int cases = 0;
  // 全ての赤/青/黄配置。赤と黄の同一辺だけは通過色が曖昧なので除外。
  for (int rx=0; rx<6; ++rx) for (int ry=0; ry<5; ++ry)
  for (int bx=0; bx<5; ++bx) for (int by=0; by<6; ++by)
  for (int yx=0; yx<6; ++yx) for (int yy=0; yy<5; ++yy) {
    if (rx==yx && ry==yy) continue;
    GateSeg r({rx},{ry,ry+1}), b({bx,bx+1},{by}), y({yx},{yy,yy+1});
    // 配置ごとに初期方位を変え、共有端点・境界・3周の接続も検証。
    if (!runCase(r,b,y,cases%4,3)) {
      printf("FAIL course=%d red=(%d,%d)-(%d,%d) blue=(%d,%d)-(%d,%d) yellow=(%d,%d)-(%d,%d) sequence=",
        ETROBO_COURSE_L,rx,ry,rx,ry+1,bx,by,bx+1,by,yx,yy,yx,yy+1);
      for(int c: crossed) printf("%c", "RBY"[c]);
      printf(" errors=%d\n",errors); return 1;
    }
    ++cases;
  }
  // 不正配置はゲート通過に数えない。
  Nansho3Scene invalid;
  invalid.gates_since_gyro_reset_ = 0;
  invalid.runRallyLap(GateSeg({1},{1,3}), GateSeg({1,2},{2}), GateSeg({4},{3,4}));
  if (invalid.gates_since_gyro_reset_ != 0) return 2;
  printf("PASS course=%d: %d configurations x 3 laps\n",ETROBO_COURSE_L,cases);
}
'''
with tempfile.TemporaryDirectory(prefix='nansho3-order-') as temp:
    cpp = Path(temp) / 'test.cpp'
    cpp.write_text(prefix + header + harness[:harness.index('int main()')] + '\n'.join(functions) + harness[harness.index('int main()'):])
    for course in (0, 1):
        binary = Path(temp) / f'test-{course}'
        subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra', '-Wno-misleading-indentation', f'-DETROBO_COURSE_L={course}', str(cpp), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
