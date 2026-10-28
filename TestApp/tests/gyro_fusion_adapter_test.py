#!/usr/bin/env python3
"""実Gyro.cppをモック機器で実行し、reset・観測源切替・欠測・再基準化を確認。"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
gyro = root / 'Common/Gyro'
headers = {
    'kernel.h': r'''
#pragma once
#include <stdint.h>
extern "C" {
typedef uint64_t SYSTIM;
#define E_OK 0
int get_tim(SYSTIM *time);
int dly_tsk(unsigned usec);
int loc_cpu(void);
int unl_cpu(void);
}
''',
    'spike/hub/imu.h': r'''
#pragma once
extern "C" {
void hub_imu_init(void);
void hub_imu_set_tilt(float);
bool hub_imu_is_ready(void);
bool hub_imu_is_stationary(void);
void hub_imu_reset_heading(void);
float hub_imu_get_heading(void);
}
''',
    'pbdrv/imu.h': r'''
#pragma once
#define PBIO_SUCCESS 0
struct pbdrv_imu_dev_t {};
struct pbdrv_imu_config_t {};
extern "C" {
int pbdrv_imu_get_imu(pbdrv_imu_dev_t **dev, pbdrv_imu_config_t **config);
bool pbdrv_imu_is_stationary(pbdrv_imu_dev_t *dev);
}
''',
    'pbio/imu.h': r'''
#pragma once
#define PBIO_IMU_HEADING_TYPE_1D 1
extern "C" float pbio_imu_get_heading(int);
''',
    'Straight.h': r'''
#pragma once
#include <stdint.h>
extern void (*encoder_read_hook)();
struct Straight {
  int32_t left = 0, right = 0;
  bool valid = true;
  bool stopped = true;
  bool stopCommanded() const { return stopped; }
  bool readEncoders(int32_t &l, int32_t &r) const {
    l = valid ? left : 0; r = valid ? right : 0;
    if (encoder_read_hook) {
      const auto hook = encoder_read_hook;
      encoder_read_hook = nullptr;
      hook();
    }
    return valid;
  }
};
''',
}
main = r'''
#include "Gyro.h"
#include "Straight.h"
#include "kernel.h"
#include <pbdrv/imu.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
Straight gMotors;
void (*encoder_read_hook)() = nullptr;
static Gyro *active_gyro = nullptr;
static void interruptReset() { active_gyro->reset(11); }
static SYSTIM now = 0;
static float raw1d = 0, raw3d = 0;
static bool stationary = true, locked = false;
extern "C" {
int get_tim(SYSTIM *time) { assert(!locked); *time = now; return E_OK; }
int dly_tsk(unsigned usec) { now += usec; return E_OK; }
int loc_cpu() { assert(!locked); locked = true; return E_OK; }
int unl_cpu() { assert(locked); locked = false; return E_OK; }
void hub_imu_init() {}
void hub_imu_set_tilt(float) {}
bool hub_imu_is_ready() { return true; }
// ブレーキ保持中は上位APIがfalseでもセンサ静止なら学習できる。
bool hub_imu_is_stationary() { return false; }
int pbdrv_imu_get_imu(pbdrv_imu_dev_t **dev, pbdrv_imu_config_t **config) {
  static pbdrv_imu_dev_t device;
  static pbdrv_imu_config_t settings;
  *dev = &device; *config = &settings; return PBIO_SUCCESS;
}
bool pbdrv_imu_is_stationary(pbdrv_imu_dev_t *) { return stationary; }
void hub_imu_reset_heading() { raw1d = raw3d = 0; }
float hub_imu_get_heading() { return raw3d; }
float pbio_imu_get_heading(int) { return raw1d; }
}
static void near(float a, float b, float tolerance = 0.001f) {
  if (fabsf(a-b) > tolerance) { fprintf(stderr,"%f != %f\n",a,b); assert(false); }
}
static void tick(Gyro &gyro, float delta, int left = 0, int right = 0) {
  now += 10000; raw1d += delta; raw3d += 2;
  gMotors.left += left; gMotors.right += right;
  gyro.updateFusion();
}
int main() {
  Gyro gyro;
  gyro.init(); assert(gyro.fusionEnabled());
  for(int i=0;i<2000;++i) tick(gyro,0.002f);
  near(gyro.fusionState().bias,0.2f,0.02f);
  assert(gyro.fusionState().stationaryUpdates > 0);
  const float bias = gyro.fusionState().bias;
  gyro.reset(3); near(gyro.heading(),3); near(gyro.fusionState().bias,bias);
  gyro.setRateHeading(true); near(gyro.heading(),3);
  gyro.setRateHeading(false); near(gyro.heading(),3);
  stationary = false;
  for(int i=0;i<100;++i) tick(gyro,1.002f,2,-2);
  near(gyro.heading(),103,0.3f);
  // カウントresetを通知した区間は車輪だけ再基準化。角度は飛ばさない。
  const float before = gyro.heading();
  gMotors.left = gMotors.right = 0;
  gyro.invalidateFusionEncoders();
  tick(gyro,0.002f); near(gyro.heading(),before,0.01f);
  gMotors.valid = false;
  tick(gyro,1.002f,2,-2);
  gMotors.valid = true;
  tick(gyro,1.002f,2,-2);
  tick(gyro,1.002f,2,-2);
  near(gyro.heading(),before+3,0.05f);
  const float after = gyro.heading();
  gyro.reset(-7); near(gyro.heading(),-7); // 原点だけ変更しbiasは維持
  near(gyro.fusionState().bias,bias,0.02f);
  assert(after > before);
  // センサ取得途中の高優先度resetを再現。古い観測でreset後の状態を上書きしない。
  active_gyro = &gyro;
  encoder_read_hook = interruptReset;
  tick(gyro, 1.002f, 2, -2);
  near(gyro.heading(), 11);
  tick(gyro, 1.002f, 2, -2);
  near(gyro.heading(), 12, 0.03f);
  assert(!locked);
  puts("gyro_fusion_adapter_test: PASS (real Gyro.cpp, reset, source continuity, encoder reset and missing data)");
}
'''
with tempfile.TemporaryDirectory(prefix='gyro-fusion-') as temp:
    temp = Path(temp)
    for name, content in headers.items():
        path = temp / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    cpp = temp / 'test.cpp'
    binary = temp / 'test'
    cpp.write_text(main)
    subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-I'+str(temp), '-I'+str(gyro), str(gyro/'Gyro.cpp'), str(cpp),
                    '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
    # 無効化時も実ソースをコンパイルし、従来の1D/3D切替へ戻せることを確認する。
    (temp / 'Gyro.cpp').write_text((gyro / 'Gyro.cpp').read_text())
    (temp / 'HeadingFusionSettings.h').write_text(
        (gyro / 'HeadingFusionSettings.h').read_text().replace(
            'constexpr bool Enabled = true;', 'constexpr bool Enabled = false;'))
    legacy = main[:main.index('int main()')] + r"""
int main() {
  Gyro gyro;
  gyro.init(); assert(!gyro.fusionEnabled());
  raw3d = 10; near(gyro.heading(), 10);
  gyro.addHeadingCorrection(2); near(gyro.heading(), 12);
  raw1d = 200; gyro.setRateHeading(true); near(gyro.heading(), 12);
  raw1d += 1; near(gyro.heading(), 13);
  gyro.setRateHeading(false); near(gyro.heading(), 13);
  gyro.reset(-7); near(gyro.heading(), -7);
  gyro.updateFusion(); near(gyro.heading(), -7);
  assert(!locked);
  puts("gyro_fusion_adapter_test: PASS (disabled legacy mode)");
}
"""
    cpp.write_text(legacy)
    subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra',
                    '-Wno-unused-function', '-Werror', '-I'+str(temp), '-I'+str(gyro),
                    str(temp/'Gyro.cpp'), str(cpp), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
