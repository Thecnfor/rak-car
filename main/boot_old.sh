#！ /bin/bash

readonly rclocal_str='''
[Unit]
Description=python boot service
After=multi-user.target

[Service]
Type=forking
User=jetson
ExecStart=/bin/sh -c "python qqq.py &"
WorkingDirectory=/home/jetson/workspace/vehicle_wbt/main/

[Install]
WantedBy=multi-user.target
'''

tar_file=/etc/systemd/system/py_boot.service
touch $tar_file
echo '# Copyright (c), WOBOT CORPORATION.  All rights reserved.'>$tar_file
# 将字符串按照换行符进行分割并打印出来
echo "$rclocal_str" | while IFS= read -r line; do
    echo "$line">>$tar_file
done
chmod 777 $tar_file

systemctl daemon-reload
systemctl enable py_boot.service
