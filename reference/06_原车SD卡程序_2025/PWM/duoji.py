import pigpio as pio
import time

pwm_pin = 12
#12底盘舵机 中值 82 右62 左102   88
#22云台x 中值72
#23云台y 68-73 70 74
# 初始化GPIO
pi = pio.pi()
if not pi.connected:
    print("pigpio未启动或连接失败")
    exit(1)

pi.set_mode(pwm_pin, pio.OUTPUT)
pi.set_PWM_frequency(pwm_pin, 50)
pi.set_PWM_range(pwm_pin, 1000)  # 建议与主程序一致

print("输入舵机PWM值，输入q退出。")

while True:
    val = input("请输入PWM值: ")
    if val.lower() == 'q':
        break
    try:
        pwm_val = int(val)
        pi.set_PWM_dutycycle(pwm_pin, pwm_val)
        print(f"已设置PWM值: {pwm_val}")
        time.sleep(2)
    except ValueError:
        print("请输入有效数字或q退出。")

pi.set_PWM_dutycycle(pwm_pin, 0)  # 停止舵机
pi.stop()

#舵机中值80