"""
SPIKE-RT Hub の Nordic SPP (NUS互換) から IMU 角度行 (例: IMU_h,12.34) を受信して表示する。

使い方 (Windows PowerShell):
    py -m pip install --user bleak
    py bt_recv.py

使い方 (macOS):
    python3 -m pip install --user bleak
    python3 bt_recv.py --list
    python3 bt_recv.py
    python3 bt_recv.py --address <uuid>
    Mac では BLE の識別子は MAC アドレスではなく UUID。
    --list に出た address を --address に渡す。

オプション:
    --list          周囲の BLE デバイスを 10 秒スキャンして一覧表示のみ行う
    --name <str>    ハブ名のフィルタ (デフォルト: "Pybricks Hub")
    --address <id>  アドレス直接指定 (Windows: MAC / macOS: UUID)
    --timeout <s>   スキャンタイムアウト秒 (デフォルト 20)
"""

from __future__ import annotations

import argparse
import asyncio
import sys
from datetime import datetime
from typing import Optional

from bleak import BleakClient, BleakScanner

# SPIKE-RT(Pybricks)が実際に広告で出しているサービス UUID
PYBRICKS_SERVICE_UUID = "c5f50001-8280-46da-89f4-6d8051e4aeef"

# 実際にシリアルデータが流れる Nordic SPP-like (NUS互換) サービス
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # hub -> PC (notify)

HUB_SERVICE_UUIDS = [PYBRICKS_SERVICE_UUID, NUS_SERVICE_UUID]


def scanner_kwargs() -> dict:
    """macOS は service_uuids 無しだと広告の name/UUID が None になる。"""
    if sys.platform == "darwin":
        return {"service_uuids": HUB_SERVICE_UUIDS}
    return {}


def device_name(d, adv=None) -> str:
    name = getattr(d, "name", None) if d is not None else None
    if not name and adv is not None:
        name = getattr(adv, "local_name", None)
    return name or ""


def service_uuids_of(adv) -> list[str]:
    if adv is None:
        return []
    raw = getattr(adv, "service_uuids", None) or []
    out: list[str] = []
    for u in raw:
        if u is None:
            continue
        out.append(str(u).lower())
    return out


def is_hub_uuid(uuids: list[str]) -> bool:
    want = {u.lower() for u in HUB_SERVICE_UUIDS}
    return bool(want.intersection(uuids))


# デバイス一覧表示用の非同期関数定義。
async def list_devices(timeout: float) -> None:
    print(f"[list] scanning {timeout:.0f}s ...")
    # 見つかったデバイスの情報を格納する辞書
    seen: dict[str, tuple[str, list[str]]] = {}

    # デバイスの情報を取得するコールバック関数
    def cb(d, adv):
        if d is None:
            return
        addr = getattr(d, "address", None)
        if not addr:
            return
        seen[addr] = (device_name(d, adv), service_uuids_of(adv))

    async with BleakScanner(detection_callback=cb, **scanner_kwargs()):
        await asyncio.sleep(timeout)

    if not seen:
        print("[list] no BLE devices found")
        if sys.platform == "darwin":
            print("[list] macOS は Bluetooth 許可と、ハブが広告中かを確認してください")
        return

    print(f"[list] found {len(seen)} device(s):")
    for addr, (name, uuids) in sorted(seen.items(), key=lambda x: x[0]):
        marker = "  <-- hub?" if is_hub_uuid(uuids) else ""
        shown = name if name else "(no name)"
        print(f"  {addr}  name={shown!r}{marker}")


async def run(name_filter: str, address: Optional[str], timeout: float) -> None:
    scan_kw = scanner_kwargs()
    device = None

    if address:
        print(f"[scan] connecting directly to {address} ...")
        device = await BleakScanner.find_device_by_address(
            address, timeout=timeout, **scan_kw
        )
    if device is None:
        print(f"[scan] searching for hub (name~={name_filter!r} or Pybricks/NUS service) ...")

        def match(d, adv):
            if d is None:
                return False
            name = device_name(d, adv)
            if name_filter and name.lower().startswith(name_filter.lower()):
                return True
            if is_hub_uuid(service_uuids_of(adv)):
                return True
            return False

        device = await BleakScanner.find_device_by_filter(
            match, timeout=timeout, **scan_kw
        )

    if device is None:
        print("[scan] not found. hub が広告中か確認してください (--list で周囲を確認可)")
        if sys.platform == "darwin":
            print("[scan] macOS では --list の UUID を --address に渡してください (MAC アドレスではない)")
        sys.exit(1)

    shown_name = device_name(device) or "(no name)"
    print(f"[scan] found: {shown_name!r} ({device.address})")

    # 生バイトを一時保持
    buffer = bytearray()

    def handle_notify(_sender, data: bytearray) -> None:
        nonlocal buffer
        if data is None:
            return
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
