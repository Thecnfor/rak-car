#!/bin/bash

tar_file=/etc/systemd/system/py_boot.service

# 使用 here-document 写入文件，避免子 shell 问题
cat > "$tar_file" << 'EOF'
# Copyright (c), WOBOT CORPORATION.  All rights reserved.
[Unit]
Description=python boot service
After=multi-user.target

[Service]
Type=simple
User=jetson
ExecStart=/usr/bin/python -u /home/jetson/workspace/vehicle_wbt/main/qqq.py
WorkingDirectory=/home/jetson/workspace/vehicle_wbt/main/
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

# 正确权限
chmod 644 "$tar_file"

# 重新加载 systemd
systemctl daemon-reload
systemctl enable py_boot.service

echo "Service installed and enabled."