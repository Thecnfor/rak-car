#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""runtime/core/key_input.py — MC602 板上鍵 raw→bool 純函數（無依賴，可離線單測）。

設計：BoardKey 的 raw→按下 映射收斂在此單一函數。真機標定（極性 / 哪個 byte
代表按下）只需改這裡，流程側全部對 bool 編程，避免把硬體不確定性散落各處。
"""


def board_key_pressed(raw):
    """BoardKey raw → bool「是否按下」。

    raw 來自 ``BoardKey_2.no_act()``（dev_id=0x0d, format="bbb"，去掉首 byte 後
    的 2 ints tuple）。預設：任一 byte 非零 = 按下。
    ⚠️ 實際極性待真機確認 —— 若按下時 bytes 是 0（低電平有效），改這裡一行即可。
    """
    if raw is None:
        return False
    vals = raw if isinstance(raw, (tuple, list)) else [raw]
    return any(int(v) != 0 for v in vals)
