module.exports = {
  apps: [
    {
      name: "rak-car-api",
      cwd: "/home/jetson/workspace/rak-car",
      script: "/home/jetson/workspace/rak-car/runtime/server.py",
      interpreter: "/usr/bin/python3",
      exec_mode: "fork",
      instances: 1,
      autorestart: true,
      watch: false,
      max_restarts: 10,
      restart_delay: 3000,
      kill_timeout: 5000,
      env: {
        PYTHONUNBUFFERED: "1",
        RAK_CAR_BIND_HOST: "0.0.0.0",
        RAK_CAR_BIND_PORT: "5050",
        RAK_CAR_PUBLIC_HOST: "192.168.6.231",
        RAK_CAR_PUBLIC_STREAM_PORT: "5050",
        RAK_CAR_PUBLIC_STREAM_PATH: "/stream/",
        RAK_CAR_AUTO_INIT: "1",
        RAK_CAR_RESET_ARM: "1",
        RAK_CAR_RESET_X_VELOCITY: "0.04",
        RAK_CAR_RESET_POSITION_ON_INIT: "1",
        RAK_CAR_STOP_AFTER_ACTION: "0",
        RAK_CAR_INFER_AUTO_START: "1",
        RAK_CAR_INFER_POLL_INTERVAL: "1.0",
        RAK_CAR_INFER_READY_TIMEOUT: "45",
        RAK_CAR_INFER_HEALTH_TIMEOUT: "2.0",
        /* 4GB RAM 缓解：先不预热任何模型,首次调用时再加载(~400ms 冷启动 vs ~150MB*2 常驻) */
        RAK_INFER_EAGER_MODELS: "",
        /* 模型 idle 60s 就卸,腾内存给其他进程 (默认 300 太长) */
        RAK_INFER_IDLE_UNLOAD_SECONDS: "60",
        /* 抑制 Paddle C++ 端的 IR pass verbose (0=INFO, 1=WARNING, 2=ERROR, 3=FATAL) */
        GLOG_minloglevel: "2",
        FLAGS_minloglevel: "2",
        PYTHONWARNINGS: "ignore",
      },
      /* 兜底: runtime/server.py 单进程泄漏超过 1.2GB 自动 PM2 重启 (仅看父进程 RSS, infer_back_end 子进程管不到) */
      max_memory_restart: "1200M",
    },
  ],
};
