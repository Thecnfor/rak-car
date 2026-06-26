# -*- coding: utf-8 -*-
import re
import time
import threading
import os
import numpy as np
from task_func import MyTask
from log_info import logger
from car_wrap import MyCar
from tools import CountRecord
import math
import sys, os
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

from multiprocessing import Process
import time





if __name__ == "__main__":
      # kill_other_python()
    my_car = MyCar()
    my_car.STOP_PARAM = False
    my_car.