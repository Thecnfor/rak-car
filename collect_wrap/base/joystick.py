import os
import struct
import time
import threading


# by wjjook
class JoyStick:
    key_map = {
        0:"A", 1:"B", 2:"X", 3:"Y", 4:"L1", 5:"R1",
        6:"Select", 7:"Start", 8:"?",
        9:"LAxis", 10:"RAxis",
        11:"?", 12:"?", 13:"?", 14:"?"}
    axis_map = {
        0:"LAxis_x", 1:"LAxis_y", 2:"LAxis_T", 
        3:"RAxis_x", 4:"RAxis_y", 5:"RAxis_T",
        6:"+_Axis_x", 7:"+_Axis_y"}

    btn_map = {1:key_map, 2:axis_map}

    def __init__(self, test=False):
        print('avaliable devices')
        for fn in os.listdir('/dev/input'):
            if fn.startswith('js'):
                print('/dev/input/'+fn)
        self.fn = '/dev/input/js0'
        self.jsdev = open(self.fn, 'rb')
        # self.jsdev.read(19*8)
        self.stop = False
        self.stick_state_init()
        if test:
            self.key_adjust()
        self.process = threading.Thread(target=self.update, args=())
        self.process.daemon = True
        self.process.start()

    def key_adjust(self):
        while True:
            if self.stop:
                break
            val, key_type, key_id = self.read()
            if key_type !=2 and key_type !=1:
                continue
            print("type:",key_type, "ID:",key_id)

    def stick_state_init(self):
        self.key = {}
        self.stick = {}
        for val in self.key_map.values():
            self.key[val] = 0
        for val in self.axis_map.values():
            self.stick[val] = 0.0
        self.key_type = {1:self.key, 2:self.stick}

    def read(self):
        evbuf = self.jsdev.read(8)
        time, val, key_type, key_id = struct.unpack('IhBB', evbuf)
        return val, key_type, key_id
    
    def update(self):
        while True:
            if self.stop:
                break
            val, key_type, key_id = self.read()
            # print(val,key_type, key_id)
            if key_type !=2 and key_type !=1:
                continue
            if key_type == 2:
                val = val / 32767
            # print(key_type)
            name = self.btn_map[key_type]
            self.key_type[key_type][name[key_id]] = val

    def get_btn(self):
        return self.key

    def get_btn_state(self, name):
        try:
            return self.key[name]
        except Exception as e:
            return None

    def get_stick(self):
        return self.stick

    def close(self):
        self.stop = True
        self.process.join()
        self.jsdev.close()

if __name__ == "__main__":
    js = JoyStick(True)
    # while True:
    #     tt = js.get_btn_state("X")
    #     ss = js.get_stick()
    #     print(ss)
    #     if tt == 1:
    #         js.close()
    #         break
    #     time.sleep(0.5)