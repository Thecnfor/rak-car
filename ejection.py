from vehicle import MotorWrap, PoutD, ServoPwm, StepperWrap
import time, math

class Ejection():
    def __init__(self, portm, portd) -> None:
        self.motor = MotorWrap(portm, -1, type="motor_280_0", perimeter=0.06/15*8)
        self.pout = PoutD(portd)
        self.step1 = StepperWrap(1)
        self.step_rad_st = self.step1.get_rad()
        self.step1_rad_cnt = 0

    def reset(self, vel=0.05):
        rad_last = self.motor.get_rad()
        
        while True:
            self.motor.set_linear(vel)
            time.sleep(0.02)
            rad_now = self.motor.get_rad()
            if abs(rad_now - rad_last) < 0.02:
                break
            rad_last = rad_now
        
        self.motor.set_linear(0)
        
    def eject(self, x=0.1, vel=0.05):
        self.reset()
        self.pout.set(1)
        self.motor.reset()
        self.motor.set_linear(0-abs(vel))
        while True:
            self.motor.set_linear(0-abs(vel))
            if abs(self.motor.get_dis()) > x:
                break
        self.motor.set_linear(0)
        self.step1_rad_cnt += 1
        self.step1.set_rad(math.pi/5*2*self.step1_rad_cnt + self.step_rad_st)
        time.sleep(0.5)
        self.pout.set(0)

eject = Ejection(5, 4)

def reset():
    eject.reset()

def send_test(x=0.1, vel=0.1):
    eject.eject(x, vel)

def pwm_tess():
    servo_pwm = ServoPwm(7, mode=270)
    while(1):
        servo_pwm.set_angle(-90, 100)
        time.sleep(1)
        servo_pwm.set_angle(0, 100)
        time.sleep(1)
        
if __name__ == "__main__":
    import argparse
    args = argparse.ArgumentParser()
    args.add_argument('--op', type=str, default="reset")
    args = args.parse_args()
    # print(args)
    if args.op == "reset":
        reset()
    # pwm_tess()
    # if args.op == "stop":
        # punish_crimall_test("infer_back_end.py")
    send_test(0.072)
    # reset()
    # task_reset()
    # ball_test()
    # cylinder_test()
    # punish_crimall_test()
    # task = MyTask()
    # highball_test()
