#!/usr/bin/env python3
import pathlib, subprocess, re, tempfile, hashlib

# 削除前の実ソースから採取した左右コース6周の動作列と比較する。
# モータ境界のみ模擬し、経路計画・方位更新・固定補正・停止順は実装を抽出する。
root = pathlib.Path(__file__).resolve().parents[1] / 'Scene/Nansho3'
names='snapTo90 facingIndex facingDegFromIndex inGrid isYAdjacent isXAdjacent isOrthoAdjacent nearerGateEnd clearBlocked blockPoint blockGate blockFutures bfsPlan bfsCost planStartsWithFwd passGateStartsWithFwd goTo passGateShortest runRallyLap prepareNextLap absoluteCardinalDeg straightTargetDeg addFixedTurnCorrection turnByAction driveOneCell beepError'.split()
prefix='''#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
struct Gyro { static float wrapDeg180(float v){while(v>=180)v-=360;while(v< -180)v+=360;return v;} };
struct Motors {void coast(){puts("coast");} void setTargetHeading(float v){printf("target %.9g\\n",v);} } gMotors;
struct LineTracer {void setEnabled(bool v){printf("lt %d\\n",v);} } gLineTracer;
void dly_tsk(unsigned v){printf("wait %u\\n",v);}
static constexpr int kDx[4]={1,0,-1,0};
#if ETROBO_COURSE_L
static constexpr int kDy[4]={0,1,0,-1};
#else
static constexpr int kDy[4]={0,-1,0,1};
#endif
'''
stubs='''void Nansho3Scene::driveCellDistance(bool stop){printf("forward %d %d %.9g stop=%d\\n",x_,y_,facing_deg_,stop);straight_running_=!stop;}
void Nansho3Scene::turnToAbsolute(float t,float e,int s){printf("turn %.9g %.9g %d\\n",t,e,s);}
'''
main='''int main(){Nansho3Scene n;
#if ETROBO_COURSE_L
GateSeg red={{3},{3,4}},blue={{1,2},{2}},yellow={{4},{4,5}};
#else
GateSeg red={{4},{4,5}},blue={{0,1},{2}},yellow={{2},{3,4}};
#endif
for(int lap=0;lap<6;++lap){if(lap)n.prepareNextLap(red,blue,yellow);n.runRallyLap(red,blue,yellow,lap<5);printf("lap %d position %d %d %.9g\\n",lap,n.x_,n.y_,n.facing_deg_);}}
'''
expected = {(0, 0): '3a67c0e795d69ca14f85b4787891f7f9b8688c5b3533d49cdfc10a21076cec26', (0, 1): 'f73a212f5d5ca8e77e22150840c0c07b4980e21d22761bd35e1f6b3386bf1bca', (1, 0): '2d6c0b30b5b7b38cbb1cd95e78712fade7d88df546968203a5634c484566fae8', (1, 1): '6f0ac6b43eafb3ed0782ab5de4f63862a6ca985aa666c650c4af152943ae3164'}
with tempfile.TemporaryDirectory(prefix='nansho3-order-') as temp:
 path = root
 s=(path/'Nansho3.cpp').read_text(); h=(path/'Nansho3.h').read_text().replace('#include <kernel.h>','').replace('#include "app.h"','').replace('private:','public:')
 fs=[]
 for name in names:
  m=re.search(r'^[^\n]*Nansho3Scene::'+name+r'\(',s,re.M)
  assert m, f'missing function: {name}'
  fs.append(s[m.start():s.index('\n}',m.end())+2])
 p=pathlib.Path(temp) / 'test.cpp';p.write_text(prefix+h+stubs+'\n'.join(fs)+main)
 for c in (0,1):
  for mode in (0,1):
   exe=str(pathlib.Path(temp) / f'test-{c}-{mode}')
   subprocess.run(['g++','-std=c++11','-O2',f'-DETROBO_COURSE_L={c}',f'-DNANSHO3_USE_TURN_DRIFT={mode}',str(p),'-o',exe],check=True)
   output = subprocess.check_output([exe])
   actual = hashlib.sha256(output).hexdigest()
   if actual != expected[(c, mode)]:
    print(output.decode())
    raise AssertionError(f'course={c} mode={mode}: motion sequence changed ({actual})')
   print(f'PASS course={c} mode={mode}: 6 laps, {len(output.splitlines())} events unchanged')
