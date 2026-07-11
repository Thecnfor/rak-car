# -*- coding: utf-8 -*-
"""
任务: get_order()

对应 car_task_function.py 中的 get_order()
机械臂动作：
- reset_position()
- move_x_position / move_y_position
- set_hand_angle / set_arm_angle
- grasp(True/False)
"""

import time
from car_wrap_2026 import MyCar, kill_other_python


def find_goods(label, dy=-0.5):
    cls_id, det_label = my_car.move_to_detection_target(label=label, delta_y=dy)
    if det_label is not None:
        return det_label

    my_car.arm.move_x_position(0.20)
    cls_id, det_label = my_car.move_to_detection_target(label=label, delta_y=dy)
    if det_label is not None:
        return det_label

    my_car.move_for([0.15, 0, 0])
    cls_id, det_label = my_car.move_to_detection_target(label=label, delta_y=dy)
    if det_label is not None:
        return det_label

    my_car.arm.move_x_position(0.30)
    cls_id, det_label = my_car.move_to_detection_target(label=label, delta_y=dy)
    if det_label is not None:
        return det_label


def get_order():
    goods_dict = {
        "青椒": "h_qing_jiao",
        "蘑菇": "h_mo_gu",
        "芹菜": "h_qin_cai",
        "番茄": "h_fan_qie",
        "油菜": "h_you_cai",
        "豆角": "h_dou_jiao",
        "西兰花": "h_xi_lan_hua",
        "土豆": "h_tu_dou",
        "金针菇": "h_jin_zhen_gu",
    }
    text_list = []
    order_list = []

    my_car.arm.reset_position()
    my_car.lane_dis_offset(speed=0.3, dis_hold=1.5)
    cls_id, label = my_car.move_to_detection_target(delta_y=None)
    my_car.move_for([0.065, 0, 0])
    my_car.arm.move_x_position(0.23)
    my_car.arm.move_x_position(0.1, out_time=4.0)
    time.sleep(0.5)
    my_car.move_for([-0.06, 0, 0])
    cls_id, label = my_car.move_to_detection_target(delta_y=None)
    time.sleep(0.5)
    text_list.append(my_car.get_ocr(label="order"))
    my_car.beep()

    my_car.arm.move_y_position(0.2)
    my_car.arm.move_x_position(0.21)
    my_car.arm.set_hand_angle("MID")
    my_car.arm.set_arm_angle("RIGHT")
    time.sleep(0.5)
    cls_id, label = my_car.move_to_detection_target()
    time.sleep(1)
    my_car.get_detection_results()
    text_list.append(my_car.get_ocr(label="order"))
    my_car.beep()

    for text in text_list:
        if text is None:
            order_list.append(None)
            continue
        order_info = my_car.order_analysis.get_res_json(text)
        order_list.append(order_info)
    order_list.sort(key=lambda x: x["address"])

    my_car.lane_dis_offset(speed=0.3, dis_hold=0.2)
    my_car.arm.set_hand_angle(angle="DOWN")
    loc = my_car.get_odometry(True)

    my_car.set_storage(True)
    my_car.arm.move_y_position(0.2)
    my_car.arm.move_x_position(0.30)
    cls_id, label = my_car.move_to_detection_target(delta_y=None)
    goods_now = order_list[1]["goods"]
    find_goods(goods_dict[goods_now])
    time.sleep(0.5)
    my_car.arm.grasp(True)
    my_car.arm.move_y_position(0.05)
    time.sleep(0.5)
    my_car.arm.move_y_position(0.2)
    my_car.arm.move_x_position(0.0)
    my_car.arm.move_y_position(0.09)
    time.sleep(0.5)
    my_car.arm.grasp(False)

    my_car.move_to_position(loc)
    my_car.arm.move_y_position(0.2)
    my_car.arm.move_x_position(0.30)
    cls_id, label = my_car.move_to_detection_target(delta_y=None)
    goods_now = order_list[0]["goods"]
    find_goods(goods_dict[goods_now])
    time.sleep(0.5)
    my_car.arm.grasp(True)
    my_car.arm.move_y_position(0.05)
    time.sleep(0.5)
    my_car.arm.move_y_position(0.2)
    my_car.arm.move_x_position(0.0)
    my_car.arm.move_y_position(0.14)
    time.sleep(0.5)
    my_car.arm.grasp(False)

    my_car.move_to_position(loc)
    return order_list


if __name__ == "__main__":
    from init import init
    init(reset_arm=True)
    get_order()