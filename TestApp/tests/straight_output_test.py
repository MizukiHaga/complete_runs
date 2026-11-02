"""実ソースの出力配分を抽出し、全パワー・補正範囲で左右対称性を検証。"""
from pathlib import Path
import subprocess
import tempfile
source = (Path(__file__).resolve().parents[1] / 'Common/Straight/Straight.cpp').read_text()
start = source.index('  const int signed_diff =')
end = source.index('\n  if (correction_debug_)', start)
code = '''#include <cassert>
struct Output { int l, r; };
Output output(int power, int diff, bool encoder_straight_) {
 int left_power=power, right_power=power;
 int travel_sign=power>=0?1:-1;
''' + source[start:end] + '''
 return {out_left,out_right};
}
int main() {
 for(int p=-100;p<=100;++p) for(int d=-40;d<=40;++d) for(int e=0;e<=1;++e) {
  auto a=output(p,d,e), b=output(p,-d,e);
  assert(a.l==b.r && a.r==b.l);
  assert(a.r-a.l==(e && p<0?-d:d));
  assert(a.l>=-100 && a.l<=100 && a.r>=-100 && a.r<=100);
  if(d==0) assert(a.l==p && a.r==p);
 }
 auto left=output(100,1,false), right=output(100,-1,false);
 assert(left.l==99 && left.r==100);
 assert(right.l==100 && right.r==99);
}
'''
with tempfile.TemporaryDirectory() as temp:
    cpp=Path(temp)/'test.cpp'; binary=Path(temp)/'test'
    cpp.write_text(code)
    subprocess.run(['g++','-std=c++11',str(cpp),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
print('PASS: output symmetry, correction difference, limits, forward/reverse')
