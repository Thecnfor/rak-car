"""main/arm/test/_runtime_guard.py
runtime 通信前置/后置守卫 —— 集中处理 localhost / ConnectionError /
ConnectTimeout / ReadTimeout,让 test_*.py 不再各自 catch 异常后 print 一坨不可读的 trace。

三种信号分清楚:
  [ABORT]  脚本主动退出 (localhost 跑 / runtime 没初始化)
  [FAIL]   网络/服务层问题 (runtime 不可达) —— 不是代码 bug
  [OK  ]   健康,可以继续
"""
import requests


def _resolve_http(client):
    """ArmClient 有 .http 包装,这里 unwrap 成 RuntimeApiClient。"""
    return getattr(client, "http", client)


def preflight(client, require_initialized: bool = True) -> bool:
    """跑 test 前调。返回 True 表示 ready to run。

    Args:
        client: RuntimeApiClient 或 ArmClient 都能传(后者自动 .http)
        require_initialized: True 时 state.initialized != True 视为未就绪

    Prints:
        一段"实时摘要" + 任意 [ABORT]/[FAIL] 提示后 return False
    """
    http = _resolve_http(client)
    origin = http.settings.server_origin

    # ---- 本地 localhost:本机没起 runtime,通常是 dev 漏设 env ----
    if "127.0.0.1" in origin or "localhost" in origin:
        print(f"[ABORT] SERVER_ORIGIN={origin} 是 localhost,本机没起 runtime")
        print('           PowerShell: $env:RAK_CAR_SERVER_ORIGIN = "http://<ip>"')
        return False

    base = http.api_base
    print(f"=== runtime preflight ({base}) ===")
    try:
        h = http.get_health()
    except requests.exceptions.ConnectionError as e:
        # 主机在但 5050 没人监听,或主机本身不可达 (UDP 也不通)
        print(f"[FAIL] runtime 在 {base} 连不上 —— 不是代码 bug,是网络/服务层问题")
        print(f"       异常: {type(e).__name__}: {str(e)[:120]}")
        print( "       可能原因:")
        print( "       1) Jetson 掉电 / 网线松")
        print( "       2) runtime 服务没起来:ssh 进去 pm2 restart rak-car-api")
        print(f"       3) IP 写错了 — 当前默认 {base},要覆盖:")
        print( '          PowerShell: $env:RAK_CAR_SERVER_ORIGIN = "http://<ip>"')
        return False
    except (requests.exceptions.ConnectTimeout, requests.exceptions.ReadTimeout) as e:
        # 5s connect 超时(motor 6 / API hang 等)
        print(f"[FAIL] runtime {base} TCP/读超时 —— 主机在但没响应")
        print(f"       异常: {type(e).__name__}: {str(e)[:80]}")
        return False
    except Exception as e:
        print(f"[FAIL] health 异常: {type(e).__name__}: {str(e)[:80]}")
        return False

    s = h.get("state", {})
    if require_initialized and not s.get("initialized"):
        print(f"[ABORT] runtime 未初始化  (state.initialized != True)")
        print(f"        last_error = {s.get('last_error')}")
        return False
    cs = s.get("controller_session", {})
    print(f"  [OK  ] initialized={s.get('initialized')}  "
          f"initializing={s.get('initializing')}  "
          f"last_error={s.get('last_error')}  "
          f"ctrl_state={cs.get('state')}  "
          f"usb_present={cs.get('usb_present')}")
    return True


def postflight(client, label: str = "after") -> None:
    """跑 test 后调。best-effort,失败也不 throw、不 exit。"""
    http = _resolve_http(client)
    try:
        h = http.get_health()
        s = h.get("state", {})
        cs = s.get("controller_session", {})
        print(f"  [{label:5s}] initialized={s.get('initialized')}  "
              f"initializing={s.get('initializing')}  "
              f"last_error={s.get('last_error')}  "
              f"ctrl_state={cs.get('state')}  "
              f"usb_present={cs.get('usb_present')}")
    except Exception as e:
        print(f"  [{label:5s}] health FAIL: {type(e).__name__}: {str(e)[:80]}")
