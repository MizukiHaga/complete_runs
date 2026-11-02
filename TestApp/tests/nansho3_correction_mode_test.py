"""実装から補正関数を抽出し、両モード・両コースの排他性を確認する。"""
from pathlib import Path
import subprocess
import tempfile

app = Path(__file__).resolve().parents[1]
source = (app / 'Scene/Nansho3/Nansho3.cpp').read_text()
header = (app / 'Scene/Nansho3/Nansho3.h').read_text()
header = header.replace('#include <kernel.h>', '').replace('#include "app.h"', '').replace('private:', 'public:')
functions = []
for name in ['snapTo90', 'absoluteCardinalDeg', 'straightTargetDeg',
             'addFixedTurnCorrection', 'turnToAbsolute']:
    pos = source.index('Nansho3Scene::' + name + '(')
    start = source.rfind('\n', 0, pos) + 1
    end = source.index('\n}\n', pos) + 3
    functions.append(source[start:end])
stubs = r'''
#include <cmath>
#include <cassert>
struct Gyro {
  static float wrapDeg180(float x) { while(x>=180)x-=360; while(x< -180)x+=360; return x; }
  void setRateHeading(bool) {}
} gGyro;
struct Motors {
  bool drift=false;
  float early=0;
  int power=0;
  void turnToHeading(float,float,int p,int,unsigned,float,int,float e,bool,int,bool d) {
    drift=d; early=e; power=p;
  }
} gMotors;
'''
main = r'''
int main() {
  Nansho3Scene s;
  s.cardinal_offset_=0;
  for(bool left : {true,false}) {
    float before=s.heading_add_;
    for(int i=0;i<s.TurnsPerHeadingAdd;++i) s.addFixedTurnCorrection(left);
    float expected=before;
    if(!NANSHO3_USE_TURN_DRIFT) expected=Gyro::wrapDeg180(before+(left?s.PerLeftTurnHeadingAddDeg:s.PerRightTurnHeadingAddDeg));
    assert(std::fabs(s.heading_add_-expected)<0.00001f);
    s.turnToAbsolute(left?-90:90, left?s.CoreTurnLeftEarlyExtraDeg:s.CoreTurnRightEarlyExtraDeg,left?-1:1);
    assert(gMotors.drift==bool(NANSHO3_USE_TURN_DRIFT));
    assert(gMotors.power==s.CoreTurnPower);
    assert(gMotors.early==s.CoreTurnEarlyStopDeg+(left?s.CoreTurnLeftEarlyExtraDeg:s.CoreTurnRightEarlyExtraDeg));
  }
  assert(std::fabs(s.straightTargetDeg(90)-Gyro::wrapDeg180(90+s.HeadingOffsetDeg+s.heading_add_))<0.0001f);
}
'''
with tempfile.TemporaryDirectory(prefix='nansho3-mode-') as temp:
    cpp=Path(temp)/'test.cpp'
    cpp.write_text('#include <initializer_list>\n'+header+stubs+'\n'.join(functions)+main)
    for course in (0,1):
        for mode in (0,1):
            binary=Path(temp)/f'test-{course}-{mode}'
            subprocess.run(['g++','-std=c++11',f'-DETROBO_COURSE_L={course}',f'-DNANSHO3_USE_TURN_DRIFT={mode}',str(cpp),'-o',str(binary)],check=True)
            subprocess.run([str(binary)],check=True)
print('PASS: fixed/TurnDrift x L/R course')
