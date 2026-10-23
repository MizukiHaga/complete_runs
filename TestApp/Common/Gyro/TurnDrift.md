# 停止後の信頼度による方位補正

左右輪の開始時からの回転量の絶対値平均が180°以上になった最初のサンプルを使う。
その時点の左右エンコーダ回転量、ジャイロ累積角、開始からの経過時間を同時に保存する。
旋回中と停止後SettleMs=500msの待機中に監視し、一度保存した測定値は更新しない。
補正は待機終了後に一度だけ適用する。待機終了まで180°に達しなければ補正しない
（DRIFT_statusはWarmingUpのまま）。サンプリング間隔による180°超過分は実測値を使う。
SampleEncoderDegは採用閾値、EncoderPer90Degは角度換算係数で、別々の設定。
ログ用のシーン側待機も残っているため、ログ有効時は追加で待つ。

```
エンコーダ角 = 方向 × 左右輪回転量平均 × 90 / EncoderPer90Deg
角度差 = エンコーダ角 − 補正前ジャイロ累積角
scale = max(1, |エンコーダ角| / 90)
mismatch = |L-R| / max(L,R)
P = (旋回方向のGyroStdDevDeg × scale)^2
R = (旋回方向のEncoderStdDevDeg × scale)^2 × (1 + 9 × (mismatch / MaxWheelMismatch)^2)
K = P / (P + R)
補正量 = clamp(K × 角度差, ±MaxCorrectionDeg)
```

標準偏差はTurnDriftConfigの設定値を使う。
左旋回はLeftGyroStdDevDeg/LeftEncoderStdDevDeg、右旋回はRightGyroStdDevDeg/RightEncoderStdDevDegを使う。
追加の左右倍率は使わない。DRIFT_gainは方向別の標準偏差から計算したK。
Enabled=falseの場合は両方向とも補正しない。
左右輪一致時のKはP/(P+R)。左右輪不一致でエンコーダの分散を増やし重みを下げる。
このペナルティも経験則であり、左右輪一致だけでは滑りや換算精度を保証できない。
ジャイロ標準偏差を大きくすると補正が強くなり、エンコーダ標準偏差を大きくすると弱くなる。
実角度との繰り返し比較で換算倍率・偏りを校正してから分散を調整する。

これは旋回単位の分散による重み付けで、旋回間で共分散を更新するカルマンフィルタではない。
一定の換算誤差やバイアスを推定・除去するものではなく、同方向への累積は残り得る。
補正は各旋回で一度だけ適用し、次の直進と旋回へ引き継ぐ。
同一インスタンスへの再適用は行わず、旋回開始時にインスタンスを初期化する。
WindowSec/TimeConstantSec/MaxUpdateGapSec は廃止。

異常値、左右輪不整合、逆方向、低速、角度差過大、平均速度差過大は棄却する。
平均速度は開始から180°到達サンプルまでの経過時間で計算する。
Enabled=false では待機と補正を行わない。エンコーダ停止テストには適用しない。

DRIFT_gain=K、DRIFT_err=180°到達サンプルの角度差、DRIFT_corr=適用量、DRIFT_rem=未適用分。
部分補正なので残差が0である必要はない。DRIFT_rateは0。
既存のstatus番号とwarm/delay列は互換性のため維持するがdelayは使用しない。warmは閾値未到達時にも維持される。
停止指令時スナップショットは補正前のまま維持する。

検証（workspaceから）:

```
g++ -std=c++11 -Wall -Wextra -Werror TestApp/tests/turn_drift_test.cpp -o /tmp/turn_drift_test
/tmp/turn_drift_test
make img=TestApp
```
