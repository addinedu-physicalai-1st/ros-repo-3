import RPi.GPIO as GPIO
import time

from pinky_lcd import LCD
from battery import Battery
from PIL import Image, ImageSequence, ImageDraw, ImageFont
import os, time

result = open('/sys/class/net/eth0/address').read().strip()
mac_address = result.strip().replace(":", "")

ssid = f"pinky_{mac_address[-4:]}"
password = "pinkypro"

text = f"SSID: {ssid}\nPW: {password}"

BUZZER_PIN = 4

C5 = 523
E5 = 659
G5 = 784

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

pwm = GPIO.PWM(BUZZER_PIN, C5)    
pwm.start(90)

pwm.ChangeFrequency(C5)
time.sleep(0.08)

pwm.ChangeFrequency(E5)
time.sleep(0.08)

pwm.ChangeFrequency(G5)
time.sleep(0.3)

pwm.stop()
GPIO.cleanup(BUZZER_PIN)

lcd = LCD()
lcd.clear()

img_width, img_height = 320, 240
background_color = (0, 0, 0)
text_color = (255, 255, 255)

img = Image.new('RGB', (img_width, img_height), color=background_color)
draw = ImageDraw.Draw(img)

try:
    font = ImageFont.truetype("/home/pinky/ap/ProggyCrossed_Regular.ttf", 30) # 글자크기 30으로 설정
    font_battery = ImageFont.truetype("/home/pinky/ap/ProggyCrossed_Regular.ttf", 20)

except:
    font = ImageFont.load_default()

battery = Battery()

battery_level = battery.battery_percentage()
battery_voltage = battery.get_voltage()
battery_text = f"{battery_level}% ({round(battery_voltage, 2)}V)"

# 배터리 텍스트 크기 계산
bbox_battery = draw.textbbox((0, 0), battery_text, font=font_battery)
battery_text_width = bbox_battery[2] - bbox_battery[0]
battery_text_height = bbox_battery[3] - bbox_battery[1]
battery_text_top_offset = bbox_battery[1]

# 배터리 아이콘 크기 설정
icon_width = 30
icon_height = 15
icon_padding = 5
icon_cap_width = 3
icon_cap_height = 6
icon_border = 2

# 아이콘 + 패딩 + 텍스트의 총 너비 계산
total_width = icon_width + icon_cap_width + icon_padding + battery_text_width
x_start = (img_width - total_width) // 2

y_icon_top = 15 # 화면 상단에서 15px 여백
y_icon_bottom = y_icon_top + icon_height
y_icon_center = y_icon_top + (icon_height / 2)

# 2a. 배터리 아이콘 몸체 그리기 (테두리)
icon_x1 = x_start
icon_y1 = y_icon_top
icon_x2 = x_start + icon_width
icon_y2 = y_icon_bottom
draw.rectangle([icon_x1, icon_y1, icon_x2, icon_y2], outline=text_color, width=icon_border)

# 2b. 배터리 아이콘 캡(+, 꼭지) 그리기
cap_x1 = icon_x2
cap_y1 = y_icon_top + (icon_height - icon_cap_height) // 2
cap_x2 = cap_x1 + icon_cap_width
cap_y2 = cap_y1 + icon_cap_height
draw.rectangle([cap_x1, cap_y1, cap_x2, cap_y2], fill=text_color)

# 2c. 배터리 잔량 채우기
fill_margin = icon_border + 1
fill_max_width = icon_width - (fill_margin * 2)
fill_width = max(0, fill_max_width * (battery_level / 100.0))

if fill_width > 0:
    fill_x1 = icon_x1 + fill_margin
    fill_y1 = icon_y1 + fill_margin
    fill_x2 = fill_x1 + fill_width
    fill_y2 = icon_y2 - fill_margin
    draw.rectangle([fill_x1, fill_y1, fill_x2, fill_y2], fill=text_color)

# 2d. 배터리 텍스트 그리기
x_text = x_start + icon_width + icon_cap_width + icon_padding
y_text_final = y_icon_center - (battery_text_height / 2) - battery_text_top_offset
draw.text((x_text, y_text_final), battery_text, fill=text_color, font=font_battery)

# 이미지 가운데 좌표 계산 후 출력
bbox = draw.textbbox((0, 0), text, font=font)
text_width = bbox[2] - bbox[0]
text_height = bbox[3] - bbox[1]
x = (img_width - text_width) // 2
y = (img_height - text_height) // 2

draw.text((x, y), text, fill=text_color, font=font)

lcd.img_show(img)
time.sleep(60)
lcd.close()
