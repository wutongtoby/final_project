# to identify number by outside call
import pyb
import sensor, image, time, os, tf, math

RED_LED_PIN = 1
BLUE_LED_PIN = 3
GREEN_LED_PIN = 2
pyb.LED(GREEN_LED_PIN).on()
pyb.LED(RED_LED_PIN).off()
pyb.LED(BLUE_LED_PIN).off()

uart = pyb.UART(3,9600,timeout_char=1000)
uart.init(9600,bits=8,parity = None, stop=1, timeout_char=1000)


def image_classification():
    pyb.LED(GREEN_LED_PIN).off()
    pyb.LED(BLUE_LED_PIN).on()
    sensor.reset()                         # Reset and initialize the sensor.
    sensor.set_pixformat(sensor.RGB565)    # Set pixel format to RGB565 (or GRAYSCALE)
    sensor.set_framesize(sensor.QVGA)      # Set frame size to QVGA (?x?)
    sensor.set_windowing((240, 240))       # Set 240x240 window.
    sensor.skip_frames(time=2000)          # Let the camera adjust.

    labels = ['3', '4', '0', 'other']

    img = sensor.snapshot().save("number.jpg")

    for obj in tf.classify('/model_demo.tflite',img, min_scale=1.0, scale_mul=0.5, x_overlap=0.0, y_overlap=0.0):
        img.draw_rectangle(obj.rect())
        img.draw_string(obj.x()+3, obj.y()-1, labels[obj.output().index(max(obj.output()))], mono_space = False)
    pyb.LED(BLUE_LED_PIN).off()
    pyb.LED(GREEN_LED_PIN).on()
    return labels[obj.output().index(max(obj.output()))]

tmp = ""

while(1):
    a = uart.readline()
    if a is not None:
        tmp += a.decode()
        print(tmp)
    #else:
     #   print("none")
    if tmp == "image_classification":
        tmp = ""
        label = image_classification()
        print(label)
        uart.write(label.encode())