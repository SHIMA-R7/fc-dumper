# -*- coding: utf-8 -*-
"""Alientek DP100 を USB HID で直接叩く。
ベンダーのアプリも仮想COMポートも要らない。

このダンパーは5V専用。VMAX_MVを緩めないこと——事故は必ず引数から入る。
"""
import time
import crcmod
import hid

VID, PID = 0x2E3C, 0xAF01
DR_H2D, DR_D2H = 0xFB, 0xFA
OP_DEVICEINFO, OP_BASICINFO, OP_BASICSET = 0x10, 0x30, 0x35
SET_MODIFY, SET_ACT = 0x20, 0x80

VMAX_MV = 5000  # このハードは5V専用。ここを緩めない

_crc = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)


def _frame(op, data=b""):
    f = bytes([DR_H2D, op & 0xFF, 0x00, len(data) & 0xFF]) + data
    c = _crc(f)
    return f + bytes([c & 0xFF, (c >> 8) & 0xFF])


def _u16(b, i):
    return b[i] | (b[i + 1] << 8)


def _parse(buf):
    if not buf or buf[0] != DR_D2H:
        return None
    n = buf[3]
    if _crc(bytes(buf[0:4 + n + 2])) != 0:
        return None
    return buf[1], bytes(buf[4:4 + n])


class DP100:
    def __init__(self):
        if hasattr(hid, "Device"):
            self.d = hid.Device(VID, PID)
            self._legacy = False
        else:
            self.d = hid.device()
            self.d.open(VID, PID)
            self._legacy = True
            self.d.set_nonblocking(0)

    def close(self):
        self.d.close()

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()

    def _read(self):
        return bytes(self.d.read(64, 500) if self._legacy
                     else self.d.read(64, timeout=500))

    def _reopen(self):
        try:
            self.d.close()
        except Exception:
            pass
        time.sleep(0.3)
        self.__init__()

    def _xfer(self, op, data=b"", want=None, tries=6, minlen=0):
        for attempt in range(tries):
            try:
                self.d.write(b'\x00' + _frame(op, data))
                time.sleep(0.06)
                r = _parse(self._read())
                if r and (want is None or r[0] == want) and len(r[1]) >= minlen:
                    return r[1]
            except OSError:
                if attempt >= tries - 2:
                    raise
                self._reopen()
        return None

    def device_info(self):
        r = self._xfer(OP_DEVICEINFO, want=OP_DEVICEINFO, minlen=22)
        if not r:
            return None
        return {"type": r[0:15].split(b"\x00")[0].decode("utf-8", "replace"),
                "hw": _u16(r, 16) / 10, "app": _u16(r, 18) / 10,
                "boot": _u16(r, 20) / 10}

    def status(self):
        r = self._xfer(OP_BASICINFO, want=OP_BASICINFO, minlen=16)
        if not r:
            return None
        return {"vin_mV": _u16(r, 0), "vout_mV": _u16(r, 2), "iout_mA": _u16(r, 4),
                "vo_max_mV": _u16(r, 6), "temp_C": _u16(r, 8) / 10,
                "out_mode": r[14], "work_st": r[15]}

    def setting(self):
        r = self._xfer(OP_BASICSET, bytes([SET_ACT]), want=OP_BASICSET, minlen=10)
        if not r:
            return None
        return {"index": r[0], "state": r[1], "vo_set_mV": _u16(r, 2),
                "io_set_mA": _u16(r, 4), "ovp_mV": _u16(r, 6), "ocp_mA": _u16(r, 8)}

    def apply(self, on, v_mV, i_mA, ovp_mV=5500, ocp_mA=1200):
        d = bytes([SET_MODIFY, 1 if on else 0,
                   v_mV & 0xFF, (v_mV >> 8) & 0xFF,
                   i_mA & 0xFF, (i_mA >> 8) & 0xFF,
                   ovp_mV & 0xFF, (ovp_mV >> 8) & 0xFF,
                   ocp_mA & 0xFF, (ocp_mA >> 8) & 0xFF])
        self._xfer(OP_BASICSET, d)
        time.sleep(0.15)
        for _ in range(4):
            c = self.setting()
            if c:
                return c
            time.sleep(0.1)
        return None


def apply_safe(p, on, v_mV, i_mA):
    if v_mV > VMAX_MV:
        raise ValueError("要求 %dmV は上限 %dmV を超えています" % (v_mV, VMAX_MV))
    return p.apply(on, v_mV, i_mA)


def power_on(p, v_mV=5000, i_mA=1000, log=print):
    """1. 出力を切ったまま書く  2. 読み戻して確認  3. 入れる  4. 実測を確認"""
    c = apply_safe(p, False, v_mV, i_mA)
    if c is None or c['vo_set_mV'] != v_mV or c['state'] != 0:
        raise RuntimeError("設定を確認できませんでした: %r" % (c,))
    log("設定確認: %.3f V(出力はまだ切)" % (c['vo_set_mV'] / 1000))
    apply_safe(p, True, v_mV, i_mA)
    time.sleep(0.4)
    s = p.status()
    v = s['vout_mV'] / 1000.0
    if v > v_mV / 1000.0 + 0.3:
        apply_safe(p, False, v_mV, i_mA)
        raise RuntimeError("出力が %.3f V と高すぎるため切りました" % v)
    log("出力ON: %.3f V / %.0f mA" % (v, s['iout_mA']))


def power_off(p, log=print):
    apply_safe(p, False, 5000, 1000)
    log("出力OFF")
