"""
SPIKE-RT Hub の Nordic SPP (NUS互換) から IMU 角度行 (例: IMU_h,12.34) を受信して表示する。

使い方 (Windows PowerShell):
    py -m pip install --user bleak
    py bt_recv.py

オプション:
    --list          周囲の BLE デバイスを 10 秒スキャンして一覧表示のみ行う
    --name <str>    ハブ名のフィルタ (デフォルト: "Pybricks Hub")
    --address <mac> MAC アドレス直接指定 (例: AA:BB:CC:DD:EE:FF)
    --timeout <s>   スキャンタイムアウト秒 (デフォルト 20)
"""

import argparse
import asyncio
import sys
from datetime import datetime

from bleak import BleakClient, BleakScanner

# SPIKE-RT(Pybricks)が実際に広告で出しているサービス UUID
PYBRICKS_SERVICE_UUID = "c5f50001-8280-46da-89f4-6d8051e4aeef"

# 実際にシリアルデータが流れる Nordic SPP-like (NUS互換) サービス
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # hub -> PC (notify)

#デバイス一覧表示用の非同期関数定義。
async def list_devices(timeout: float) -> None:
    print(f"[list] scanning {timeout:.0f}s ...")
    # 見つかったデバイスの情報を格納する辞書
    seen: dict[str, tuple[str | None, list[str]]] = {}

    # デバイスの情報を取得するコールバック関数
    def cb(d, adv):
        # デバイスの情報を取得
        seen[d.address] = (d.name or adv.local_name, list(adv.service_uuids or []))

    scanner = BleakScanner(detection_callback=cb)
    await scanner.start()
    await asyncio.sleep(timeout)
    await scanner.stop()

    if not seen:
        print("[list] no BLE devices found")
        return

# デバイス接続用の非同期関数定義。
async def run(name_filter: str, address: str | None, timeout: float) -> None:
    if address:
        print(f"[scan] connecting directly to {address} ...")
        device = await BleakScanner.find_device_by_address(address, timeout=timeout)
    else:
        print(f"[scan] searching for hub (name~={name_filter!r} or Pybricks service) ...")

        #
        def match(d, adv):
            name = (d.name or adv.local_name or "")
            if name_filter and name.startswith(name_filter):
                return True
            uuids_lower = [u.lower() for u in (adv.service_uuids or [])]
            if PYBRICKS_SERVICE_UUID in uuids_lower:
                return True
            return False

        device = await BleakScanner.find_device_by_filter(match, timeout=timeout)

    if device is None:
        print("[scan] not found. hub が広告中か確認してください (--list で周囲を確認可)")
        sys.exit(1)

    print(f"[scan] found: {device.name!r} ({device.address})")

    #生バイトを一時保持
    
    buffer = bytearray()

    def handle_notify(_sender, data: bytearray) -> None:
        nonlocal buffer
        buffer.extend(data)
        while b"\n" in buffer:
            line, _, buffer = buffer.partition(b"\n")
            text = line.decode("utf-8", errors="replace").rstrip("\r")
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            print(f"{ts}  {text}")

    async with BleakClient(device) as client:
        print("[conn] connected. subscribing NUS TX notifications...")
        await client.start_notify(NUS_TX_UUID, handle_notify)
        print("[conn] listening. Ctrl-C で終了")
        try:
            while client.is_connected:
                await asyncio.sleep(1.0)
        except KeyboardInterrupt:
            pass
        finally:
            try:
                await client.stop_notify(NUS_TX_UUID)
            except Exception:
                pass


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--name", default="Pybricks Hub")
    ap.add_argument("--address", default=None)
    ap.add_argument("--timeout", type=float, default=20.0)
    args = ap.parse_args()
    try:
        if args.list:
            asyncio.run(list_devices(args.timeout if args.timeout != 20.0 else 10.0))
        else:
            asyncio.run(run(args.name, args.address, args.timeout))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
