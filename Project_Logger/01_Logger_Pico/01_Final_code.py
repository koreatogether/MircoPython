
'''
잘 작동 되는 코드 23-05-18
sdcard 모듈을 추가해서 sd 카드에 파일 저장하기
https://microcontrollerslab.com/micro-sd-card-module-raspberry-pi-pico/
https://www.instructables.com/Raspberry-Pi-Pico-Micro-SD-Card-Interface/

DS3231 라이브러리 주소 : https://github.com/balance19/micropython_DS3231/blob/main/DS3231_example.py

함수를 사용하여 다양한 작업을 캡슐화하여 코드를 더 모듈화할 수 있습니다. 예를 들어 lm35_temperature() 함수는 별도의 파일로 이동할 수 있습니다.
오류 처리를 추가하여 코드를 더 강력하게 만들 수 있습니다. 예를 들어 sdcard_function() 함수는 SD 카드를 열거나 쓰기할 때 오류를 확인할 수 있습니다.
더 효율적인 알고리즘을 사용하여 코드를 더 효율적으로 만들 수 있습니다.
예를 들어 lm35_temperature() 함수는 전압을 온도에 변환하기 위해 더 효율적인 알고리즘을 사용할 수 있습니다.
'''


'''
WIRING

PICO BOARD                   LM35 TEMPERATURE

3.3 OUT(36 Pin)       ||            VCC
GND                   ||            GND
ADC0 Port             ||            SIG

######################################################

PICO BOARD                    SD CARD MODULE

VBUS(5V , 40 Pin)     ||           VCC
GND                   ||           GND
GP 10(SPI 1 SCK)      ||           SCK
GP 11(SPI 1 TX)       ||           MOSI
GP 12(SPI 1 RX)       ||           MISO
GP 13(SPI 1 CSn)      ||           CS

#####################################################

PICO BOARD                   DS3231 MODULE

3.3 OUT(36 Pin)       ||           VCC
GND                   ||           GND
GP 16 ( SDA 0)        ||           SDA
GP 17 ( SCL 0)        ||           SCL

######################################################

PICO BOARD                   I2C OLED MODULE

3.3 OUT(36 Pin)       ||           VCC
GND                   ||           GND
GP 18 ( SDA 1)        ||           SDA
GP 19 ( SCL 1)        ||           SCL

'''

'''
라이센스 관련
활용가능한 모든 AI 툴은 활용했고
특히 bing.com 과 chatGPT의 활용이 높았습니다.

참고로 2차 저작권 관련해서는 MIT 라이센스입니다.
'''

from machine import ADC, Pin, I2C , SPI
from utime import ticks_diff, ticks_ms
from ssd1306 import SSD1306_I2C
import os
import sdcard
import RTC_DS3231

# I2C initialization
i2c_hard = I2C(1,scl=Pin(19), sda=Pin(18))
i2c_oled = SSD1306_I2C(128, 64, i2c_hard, addr=0x3c)

# Variables for timing
last_2_sec_ticks_ms = ticks_ms()
last_3_sec_ticks_ms = ticks_ms()

# Flag to track display state
is_display_on = False

#SPI 객체 만들고 이용해서 SD 카드 객체 만들기
spi = SPI(1 , sck=Pin(10) , mosi=Pin(11), miso=Pin(12))
sd = sdcard.SDCard(spi, Pin(13)) # 객체 , CS핀
# SD 카드에 가상 파일 시스템 (VIRTUAL FILE STSTEM)을 생성
vfs = os.VfsFat(sd)
# sd 카드를 '/sd' 디렉터리로 마운트
os.mount(vfs, "/sdcard") # 파일 시스템 객체 사용
# os.mount(sd , '/sdcard') 를 통해서도 가능함

# 디렉터리 내 같은 파일 있는지 체크 있다면 숫자를 증가시킴
filename = "/sdcard/test.txt"
count = 1

rtc = RTC_DS3231.RTC()
# rtc.DS3231_SetTime(b'\x00\x14\x18\x28\x14\x07\x21')
times = rtc.DS3231_ReadTime(0) # 1번 모드도 있음  , 전역으로 뺌

while True:
    try:
        os.stat(filename)
        filename = "/sdcard/test_%d.txt" % count
        count += 1
    except OSError:
        break

# LM35 temperature sensor reading
def lm35_temperature():
    lm35_pin = Pin(26, Pin.IN)
    lm35_adc = ADC(lm35_pin)
    lm35_adc_read_value = lm35_adc.read_u16()
    lm35_volt = (lm35_adc_read_value / 65535) * 3.3
    convert_milivolt_lm35 = (lm35_volt - 0.53) * 1000
    caculate_lm35_temperature = convert_milivolt_lm35 / 10
    print("LM35 Temperature:", caculate_lm35_temperature)
    return caculate_lm35_temperature

# PICO internal temperature sensor reading
def in_temperature():
    pico_temp_sensor = ADC(4)
    VOLTAGE_CONVERSION = 3.3 / 65535
    adc_4_value = pico_temp_sensor.read_u16()
    adc_4_voltage = adc_4_value * VOLTAGE_CONVERSION
    inside_temperature = 27 - (adc_4_voltage - 0.706) / 0.001721
    print("PICO Inside Temperature:", inside_temperature)
    return inside_temperature

# OLED display function
def i2c_oled_display():
    i2c_oled.text("{}".format(times[6]), 5,5)   # year
    i2c_oled.text(" / " , 35,5)
    i2c_oled.text("{}".format(times[5]), 60,5)  # month
    i2c_oled.text(" / " , 65,5)
    i2c_oled.text("{}".format(times[4]), 90,5)  # day
    i2c_oled.text("{}".format(times[2]), 30,20) # hour
    i2c_oled.text(" : " , 40,20)
    i2c_oled.text("{}".format(times[1]), 60,20) # minute
    i2c_oled.text(" : " , 70,20)
    i2c_oled.text("{}".format(times[0]), 90,20) # second
    i2c_oled.text("LM35 : {:.2f}".format(lm35_temperature()), 0, 45)
    i2c_oled.text("PICO TEMP : {:.2f}".format(in_temperature()), 0, 55)
    i2c_oled.show()


def sdcard_function():
    # 디렉터리 내 파일 목록 출력
    print(os.listdir('/sdcard'))

    # 추가모드로 열기
    file = open(filename, "a")
    file.write("{}-{}-{}({}) {}:{}:{},{:0.1f},{:0.1f},\n".format(times[6], times[5], times[4],times[3], times[2], times[1], times[0] , lm35_temperature() , in_temperature()))    
    file.close()
    file = open(filename, "r")
    if file != 0:
        print("reading from sd card")
        read_data = file.read()
        print(read_data)
        file.close()

# RTC 모듈 값을 튜플에서 읽어와서 시리얼프린트하기
def rtc_function():
    
    print(times[0] , times[1] , times[2] , times[3] , times[4] , times[5] , times[6])

while True:    
    # 2s loop
    if ticks_diff(ticks_ms(), last_2_sec_ticks_ms) >= 2000:
        last_2_sec_ticks_ms = ticks_ms()
        print("2s loop ~")
        times = rtc.DS3231_ReadTime(0) # 안 넣으면 시간이 갱신안됨
        rtc_function()
        in_temperature()
        lm35_temperature()
        sdcard_function()
        

    # 3s loop
    if ticks_diff(ticks_ms(), last_3_sec_ticks_ms) >= 3000:
        last_3_sec_ticks_ms = ticks_ms()
        print("3s loop ~")
        if is_display_on:
            i2c_oled.fill(0) # Turn off the display by filling with black pixels
            i2c_oled.show()
            is_display_on = False
        else:
            i2c_oled_display()
            is_display_on = True




