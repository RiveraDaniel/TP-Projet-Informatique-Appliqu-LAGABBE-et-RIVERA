#MQTT SERVER

import paho.mqtt.client as mqtt
# Define event callbacks
client = mqtt.Client()
def on_connect(client, userdata, rc):
    if rc == 0:
        print("Connected successfully.")
    else:
        print("Connection failed. rc= "+str(rc))

def on_publish(client, userdata, mid):
    print("Message "+str(mid)+" published.")

def on_subscribe(client, userdata, mid, granted_qos):
    print("Subscribe with mid "+str(mid)+" received.")

def on_message(client, userdata, msg):
    print("Message received on topic "+msg.topic+" with QoS "+str(msg.qos)+" and payload "+msg.payload)

mqttclient = mqtt.Client()

# Assign event callbacks
mqttclient.on_connect = on_connect
mqttclient.on_publish = on_publish
mqttclient.on_subscribe = on_subscribe
mqttclient.on_message = on_message

# Connect
mqttclient.username_pw_set("oriverabaena@gmail.com", "58f57850")
mqttclient.connect("mqtt.dioty.co", 1883)

#PROGRAMME 

import machine, onewire, time, ds18x20, binascii
from machine import Pin, ADC
from time import sleep_ms
from neopixel import NeoPixel
from sh1106 import SH1106_I2C
from images import (attention)
i2c = machine.I2C(sda = machine.Pin(21), scl = machine.Pin(22))
n = 8                              
p = 26                             
np = NeoPixel(Pin(p), n)          
bp = Pin(25, Pin.IN)

data = Pin(18, Pin.IN)
data2 = Pin(27, Pin.IN)

ds = ds18x20.DS18X20(onewire.OneWire(data))
ds2 = ds18x20.DS18X20(onewire.OneWire(data2))

roms = ds.scan()
roms2 = ds2.scan()
print('found probes:',roms)
print('found probes:',roms2)

oled = SH1106_I2C(128, 64, i2c)
oled.fill(0)
oled.text("Bienvenidos", 0, 0)
oled.show()
oled.fill(0)
time.sleep_ms(10000)
oled.text("Bonjour", 0, 0)
oled.show()

def afficher():
    fb = framebuf.FrameBuffer(buffer,128,64,framebuf.MONO_HLSB)
    oled.blit(fb,0,0)
    oled.show()
    sleep_ms(1000)

for x in range(0,n):
    np[x] = (0, 0, 0)         
    np.write()
    
while True:
    
#TEMPERAURE AU NODERED
    ds.convert_temp()
    ds2.convert_temp()
    for rom in roms:
        print('temperature 1:',ds.read_temp(rom))
        mqttclient.subscribe("/oriverabaena@gmail.com/Capteur 1 PIN 18")  
        mqttclient.publish("/oriverabaena@gmail.com/Capteur 1 PIN 18", ds.read_temp(rom))
    for rom2 in roms2:
        print('temperature 2:', ds2.read_temp(rom2))
        mqttclient.subscribe("/oriverabaena@gmail.com/Capteur 2 PIN 27")
        mqttclient.publish("/oriverabaena@gmail.com/Capteur 2 PIN 27",ds2.read_temp(rom2))
    print()
    
#TEMPERATURE AUX NEOPIXEL ET OLED
    
    #couleur
    r=0
    g=0
    b=0
    if (ds.read_temp(rom)+ds2.read_temp(rom2))/2 <= 23:
         r=0
         g=100
         b=0
    if (ds.read_temp(rom)+ds2.read_temp(rom2))/2 > 23 and(ds.read_temp(rom)+ds2.read_temp(rom2))/2 < 31:
         r=70
         g=50
         b=0
    if (ds.read_temp(rom)+ds2.read_temp(rom2))/2 >= 31:
         r=100
         g=0
         b=0
    #lampe
    l=0
    if (ds.read_temp(rom)+ds2.read_temp(rom2))/2 <= 21:
        l=1
    if (ds.read_temp(rom)+ds2.read_temp(rom2))/2 > 21 and(ds.read_temp(rom)+ds2.read_temp(rom2))/2 <= 23:
        l=2
    if (ds.read_temp(rom)+ds2.read_temp(rom2))/2 > 23 and(ds.read_temp(rom)+ds2.read_temp(rom2))/2 <= 25:
        l=3
    if (ds.read_temp(rom)+ds2.read_temp(rom2))/2 > 25 and(ds.read_temp(rom)+ds2.read_temp(rom2))/2 <= 27:
        l=4
    if (ds.read_temp(rom)+ds2.read_temp(rom2))/2 > 27 and(ds.read_temp(rom)+ds2.read_temp(rom2))/2 <= 29:
        l=5
        buffer = bytearray(attention)
        afficher()
    if (ds.read_temp(rom)+ds2.read_temp(rom2))/2 > 29 and(ds.read_temp(rom)+ds2.read_temp(rom2))/2 <= 31:
        l=6
        buffer = bytearray(attention)
        afficher()
    if (ds.read_temp(rom)+ds2.read_temp(rom2))/2 > 31 and(ds.read_temp(rom)+ds2.read_temp(rom2))/2 <= 33:
        l=7
        buffer = bytearray(attention)
        afficher()
    if (ds.read_temp(rom)+ds2.read_temp(rom2))/2 > 33:
        l=8
        buffer = bytearray(attention)
        afficher()
    for x in range(0,l):
        np[x] = (r, g, b)         
        np.write()
        x = x + 1
    for y in range(l,n):
        np[x] = (0, 0, 0)         
        np.write()
        y = y + 1   

    time.sleep_ms(1000)

    
    








