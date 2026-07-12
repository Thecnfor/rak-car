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
        RAK_CAR_PUBLIC_HOST: "192.168.3.60",
        RAK_CAR_PUBLIC_STREAM_PORT: "5050",
        RAK_CAR_PUBLIC_STREAM_PATH: "/stream/",
        RAK_CAR_AUTO_INIT: "1",
        RAK_CAR_RESET_ARM: "0",
        RAK_CAR_RESET_POSITION_ON_INIT: "1",
        RAK_CAR_STOP_AFTER_ACTION: "0",
      },
    },
  ],
};
