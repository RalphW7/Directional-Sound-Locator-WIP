from machine import ADC
from machine import Pin, PWM
import time
import _thread
import math

adc1 = ADC(26)
adc2 = ADC(27)
adc3 = ADC(28)

servo1 = PWM(Pin(15))
servo1.freq(50)

def set_angle1(thet):
    min = 500
    max = 2500
    coef = (max - min)/3.14

    pulse = min + coef*thet
    duty = int((pulse/20000)*65535)
    servo1.duty_u16(duty)

#mic positions:
#     3
# 1   2

#mic positions in meter grid, mic 1 treated as origin
pos1 = [0,0]
pos2 = [0.077, 0]
pos3 = [0.077, 0.056]
C = 343 #speed of sound

THRESHOLD = 40000      # adjust experimentally
WAIT_WINDOW_US = 3000  # how long to wait for other mics

shared_vals = {
    "order": [],
    "delays": [],
    "angle": []
}

lock = _thread.allocate_lock()

def print_thread():
    while True:
        lock.acquire()
        order = shared_vals["order"]
        delays = shared_vals["delays"]
        theta = shared_vals["angle"]
        lock.release()

        if order:
            print("Detection order:", order)
            print("Delays (us):", delays)
            print("Theta:", theta)
        
        if (theta and theta[0] >= 0 and  theta[0] != 6.7):
            set_angle1(theta[0])
            time.sleep(1)
        

        time.sleep(0.1)

_thread.start_new_thread(print_thread, ())

dist1_2 = pos2[0] - pos2[0]
dist2_3 = pos3[1] - pos2[1]
dist1_3 = math.sqrt(dist1_2**2 + dist2_3**2)

while True:

    val1 = adc1.read_u16()
    val2 = adc2.read_u16()
    val3 = adc3.read_u16()

    # wait until any mic exceeds threshold
    if val1 > THRESHOLD or val2 > THRESHOLD or val3 > THRESHOLD:

        t0 = time.ticks_us()

        times = [None, None, None]

        if val1 > THRESHOLD:
            times[0] = t0
        if val2 > THRESHOLD:
            times[1] = t0
        if val3 > THRESHOLD:
            times[2] = t0

        start_wait = t0

        while time.ticks_diff(time.ticks_us(), start_wait) < WAIT_WINDOW_US:

            val1 = adc1.read_u16()
            val2 = adc2.read_u16()
            val3 = adc3.read_u16()

            now = time.ticks_us()

            if times[0] is None and val1 > THRESHOLD:
                times[0] = now

            if times[1] is None and val2 > THRESHOLD:
                times[1] = now

            if times[2] is None and val3 > THRESHOLD:
                times[2] = now

        # compute order and fixed mic delays
        detections = []
        for i, t in enumerate(times):
            if t is not None:
                detections.append((i + 1, t))

        detections.sort(key=lambda x: x[1])

        if detections:

            first_time = detections[0][1]

            # detection order still useful for debugging
            order = [mic for mic, _ in detections]

            # fixed indexing:
            # delays[0] = mic1
            # delays[1] = mic2
            # delays[2] = mic3
            delays = [None, None, None]

            for mic_num, t in detections:
                delays[mic_num - 1] = time.ticks_diff(t, first_time)

            if (delays[0] != None and delays[1] != None and delays[2] != None):
                delta21 = delays[1] - delays[0]
                delta31 = delays[2] - delays[0]

                a = pos2[0] - pos1[0]
                b = pos2[1] - pos1[1]
                c = pos3[0] - pos1[0]
                d = pos3[1] - pos1[1]

                b1 = C*delta21
                b2 = C*delta31

                det = a*d - b*c

                if(det != 0):
                    sx = (d*b1 - b*b2)/det
                    sy = (-c*b1 + a*b2)/det
                    norm = math.sqrt(sx**2 + sy**2)
                    if (norm != 0):
                        sx /= norm
                        sy /= norm
                        ang = math.atan2(sx, sy)
                    else:
                        ang = 6.7
                else:
                    ang = 6.7
            else:
                ang = 6.7

            lock.acquire()
            shared_vals["order"] = order
            shared_vals["delays"] = delays
            shared_vals["angle"] = [ang]
            lock.release()

        # small debounce so same sound doesn't retrigger
        time.sleep_ms(50)
