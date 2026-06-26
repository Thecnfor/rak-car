

import cv2
import threading
import time
import json
import subprocess
import os, sys
# 添加上两层目录
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))) 


from camera import Camera
from log_info import logger
from vehicle import ScreenShow, Key4Btn


class CollectData:
    def __init__(self, cap_index: int=1) -> None:
        # 获取当前存储目录
        path_dir = os.path.abspath(os.path.dirname(__file__))
        # 获取模型存储目录
        self.dir = os.path.join(path_dir, "image_set2")
        if not os.path.exists(self.dir):
            os.mkdir(self.dir)

        self.index = 0
        self.cap = Camera(cap_index, 640, 480)
        self.display = ScreenShow()
        self.run()

    def run(self):
        name_length = 4
        while True:
            img = self.cap.read()
            #img = self.cap_front.read()
            cv2.imshow("image", img)
            key_val = cv2.waitKey(0)
            # 按下esc, q退出， 按下p键s键空格拍照
            if key_val == 27 or key_val == ord("q"):
                print (1)
                break
            elif key_val == ord("p") or key_val == ord("s") or key_val ==ord(" "):
                print (2)
                self.index += 1
                img_name = (name_length - len(str(self.index)))*'0' + str(self.index) +'.jpg'
                self.display.show("\n " + img_name+"\n")
                img_path = os.path.join(self.dir, img_name)
                cv2.imwrite(img_path, img)
                #time.sleep(0.2)

        self.close()

    def close(self):
        self.display.show("control end!\nimage:{}\n".format(self.index))
        self.cap.close()


if __name__ == "__main__":
    remote_car = CollectData(1)
    # remote_car = CollectData(2)
  