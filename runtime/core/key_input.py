#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""runtime/core/key_input.py — MC602 板上鍵 raw→bool 純函數（無依賴，可離線單測）。

設計：BoardKey 的 raw→按下 映射收斂在此單一函數。真機標定（極性 / 哪個 byte
代表哪顆實體按鈕）只需改這裡或 config_car.yml，流程側全部對 bool 編程，
避免把硬體不確定性散落各處。

⚠️ MC602 板上 3 顆按鈕 vs SDK wrapper：BoardKey_2.no_act() 回傳時丟掉首 byte
（``[1:]``），目前 read() 只看到 2 個 byte。若指定按鈕對應被丟的那顆，需繞過
wrapper 直接取 DevCmdInterface 3 bytes——見真機標定腳本說明。
"""

# mode 常數（也給 config 用，避免拼字錯誤）
MODE_ANY = "any"
MODE_SPECIFIC = "specific"


def board_key_pressed(raw, mode: str = MODE_ANY, button_index: int = 0) -> bool:
    """BoardKey raw → bool「是否按下」。

    Parameters
    ----------
    raw : tuple/list/scalar or None
        來自 ``BoardKey.read()`` 的讀值（注意：當前 wrapper 回傳 2 bytes；
        若要 3 顆全見，需繞過 wrapper，見模組 docstring）。
    mode : str
        - ``"any"``：任一 byte 非零 → 按下（預設，比賽最保險）。
        - ``"specific"``：僅 ``raw[button_index]`` 非零 → 按下（指定單顆）。
    button_index : int
        當 ``mode="specific"`` 時認哪個 byte（0-based）。需真機標定後填寫。

    真機標定（10 秒）：在 Jetson 跑 ``python run.py --wait-key``，逐顆按鈕按一下
    並印 raw，找「按下時 byte N 非零」的 N，填入 ``config_car.yml`` 的
    ``io.key.button_index``。若 SDK wrapper 看不到的 byte 就是你要的那顆，
    改用 ``_board_key_pressed_raw3()``（待加）或修 wrapper。
    """
    if raw is None:
        return False
    vals = raw if isinstance(raw, (tuple, list)) else [raw]
    vals = [int(v) for v in vals]
    if mode == MODE_SPECIFIC:
        if button_index < 0 or button_index >= len(vals):
            return False
        return vals[button_index] != 0
    # mode == "any"（預設）
    return any(v != 0 for v in vals)