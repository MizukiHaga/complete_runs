# Nansho3

旧Nansho3Honbanを統合した走行実装。旋回・直進（S字加減速）・距離・ゲート設定は旧Honbanを引き継ぐ。

`TestApp/app.h` の `NANSHO3_USE_TURN_DRIFT`:

- `0`: 固定値補正（初期設定）。Nansho3.h の `PerLeftTurnHeadingAddDegR/L`、`PerRightTurnHeadingAddDegR/L` を旋回回数に応じて目標方位へ加算。
- `1`: Common/Gyro/TurnDrift.h のエンコーダ＋ジャイロ補正。固定値は加算しない。左右の重みは `EncoderWeightLeft` / `EncoderWeightRight`。カルマンフィルタではない。

どちらも Common/Straight を使用し、パワー・旋回終了条件・直進距離は共通。片輪退出旋回は従来どおり専用処理。
Test14～17の方位旋回も同じフラグで補正方式を切り替える。TurnDrift有効時は固定値を加算しない。エンコーダのみで停止する専用設定（Nansho3SpinStopOnEncoder=true）は従来の固定補正を使用する。Test17はNansho3の走行定数を参照する。Test19/20はNansho3の初期走行／直進テストを呼ぶ。


検証: `python3 TestApp/tests/nansho3_correction_mode_test.py`
