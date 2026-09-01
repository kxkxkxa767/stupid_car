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

print("输入占空比(0~40000)，输入q退出：")
while True:
    val = input("占空比: ")
    if val.lower() == 'q':
        break
    try:
        duty = int(val)
        if 0 <= duty <= 40000:
            pi.set_PWM_dutycycle(pwm_pin, duty)
            print(f"已设置占空比: {duty}")
        else:
            print("请输入0~40000之间的数值")
    except ValueError:
        print("请输入有效数字或q退出")

pi.set_PWM_dutycycle(pwm_pin, 0)  # 停止电机
pi.stop()