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

# 解锁电机
pi.set_PWM_dutycycle(pwm_pin, 10000)
time.sleep(2)

# 启动点
for dy in range(20000, 10000, -1000):
    pi.set_PWM_dutycycle(pwm_pin, dy)
    print(dy)
    time.sleep(2)

pi.set_PWM_dutycycle(pwm_pin, 0)  # 停止电机
pi.stop()



'''

# 寻找后转
 for dy in range(10000, 5000, -1000):
    pi.set_PWM_dutycycle(pwm_pin, dy)
    print(dy)
    time.sleep(2)

# 寻找前转
for dy in range(10000, 40000, 100):
    pi.set_PWM_dutycycle(pwm_pin, dy)
    print(dy)
    time.sleep(2)

'''

#9800开始到6000反转依次加快
#10200开始到18000正转依次加快