"""終了時の実装を抽出し、重み0の無変更と補正差分のみの適用を検証。"""
from pathlib import Path
import subprocess
import tempfile
app=Path(__file__).resolve().parents[1]
s=(app/'Common/Straight/Straight.cpp').read_text()
a=s.index('    if (isfinite(correction) && correction != 0.0f)')
b=s.index('\n    }',a)+6
h=(app/'Common/Gyro/TurnDrift.h').read_text()
with tempfile.TemporaryDirectory() as t:
    p=Path(t)
    for weight in ('0.0f','0.3f'):
        import re
        header=re.sub(r'(EncoderWeight(?:Left|Right) = )[^;]+',r'\g<1>'+weight,h)
        code=header+'''\n#include <cassert>
struct Sensor { float heading=123.0f; int calls=0;
 void addHeadingCorrection(float d){heading+=d;++calls;}
} gGyro;
void apply(float correction){
'''+s[a:b]+'''\n}
int main(){
 for(int sign=-1;sign<=1;sign+=2){
  TurnDrift d;
  float before=gGyro.heading;
  float c=d.update(sign*85,sign*180,-sign*180,1,sign);
  apply(c);
  assert(fabsf(gGyro.heading-before-c)<0.0001f);
  int count=gGyro.calls;
  apply(d.update(sign*85,sign*180,-sign*180,1,sign));
  assert(gGyro.calls==count);
 }
'''+('assert(gGyro.calls==0);' if weight=='0.0f' else 'assert(gGyro.calls==2);')+'\n}'
        cpp=p/'test.cpp'; cpp.write_text(code)
        subprocess.run(['g++','-std=c++11',str(cpp),'-o',str(p/'test')],check=True)
        subprocess.run([str(p/'test')],check=True)
print('PASS: zero weight leaves heading untouched; signed corrections applied once')
