import pigpio as pio
import time

pwm_pin = 13

# 初始化
pi = pio.pi()
if not pi.connected:
    print("pigpio未启动或连接失败")
    exit(1)

pi.set_mode(pwm_pin, pio.OUTPUT)
pi.set_PWM_frequency(pwm_pin, 200)
pi.set_PWM_range(pwm_pin, 40000)

# 初始占空比10000
pi.set_PWM_dutycycle(pwm_pin, 10000)
print("初始化，占空比: 10000")
time.sleep(2)

condition = 1  # 初始条件

if condition == 1:
    pi.set_PWM_dutycycle(pwm_pin, 10800)
    print("条件为1，占空比: 10200")
    time.sleep(2)
    condition = 2  # 更改条件为2
    print("条件已更改为2")

# 结束时停止电机
pi.set_PWM_dutycycle(pwm_pin, 0)
pi.stop()