"""旧Honbanの補正テスト入口。統合後の両モードを検証する。"""
from pathlib import Path
import runpy
runpy.run_path(str(Path(__file__).with_name('nansho3_correction_mode_test.py')), run_name='__main__')
