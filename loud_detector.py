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

BUFFER_SIZE = 64
SAMPLE_DELAY_US = 20
AMP_THRESHOLD = 20000


def capture_buffer():
    mic1 = []
    mic2 = []
    mic3 = []

    for _ in range(BUFFER_SIZE):
        mic1.append(adc1.read_u16())
        mic2.append(adc2.read_u16())
        mic3.append(adc3.read_u16())
        time.sleep_us(SAMPLE_DELAY_US)

    return mic1, mic2, mic3


def remove_dc(signal):
    avg = sum(signal) / len(signal)
    return [x - avg for x in signal]


def cross_correlate(sig1, sig2, max_shift=30):
    best_shift = 0
    best_score = -1e18

    for shift in range(-max_shift, max_shift + 1):
        score = 0

        for i in range(len(sig1)):
            j = i + shift

            if 0 <= j < len(sig2):
                score += sig1[i] * sig2[j]

        if score > best_score:
            best_score = score
            best_shift = shift

    return best_shift

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
        
        if (theta and theta[0] >= 0 and theta[0] != 6.7):
            set_angle1(theta[0])
            time.sleep(1)
        

        time.sleep(0.1)

_thread.start_new_thread(print_thread, ())

dist1_2 = pos2[0] - pos1[0]
dist2_3 = pos3[1] - pos2[1]
dist1_3 = math.sqrt(dist1_2**2 + dist2_3**2)

while True:

    mic1, mic2, mic3 = capture_buffer()

    amp1 = max(mic1) - min(mic1)
    amp2 = max(mic2) - min(mic2)
    amp3 = max(mic3) - min(mic3)

    peak_amp = max(amp1, amp2, amp3)

    if peak_amp > AMP_THRESHOLD:

        mic1 = remove_dc(mic1)
        mic2 = remove_dc(mic2)
        mic3 = remove_dc(mic3)

        shift12 = cross_correlate(mic1, mic2)
        shift13 = cross_correlate(mic1, mic3)

        delay12 = shift12 * SAMPLE_DELAY_US
        delay13 = shift13 * SAMPLE_DELAY_US

        t1 = 0
        t2 = delay12
        t3 = delay13

        first = min(t1, t2, t3)

        delays = [
            t1 - first,
            t2 - first,
            t3 - first
        ]

        order = sorted(
            [1, 2, 3],
            key=lambda i: delays[i - 1]
        )

        delta21 = delay12
        delta31 = delay13

        a = pos2[0] - pos1[0]
        b = pos2[1] - pos1[1]
        c = pos3[0] - pos1[0]
        d = pos3[1] - pos1[1]

        b1 = C * (delta21 / 1_000_000)
        b2 = C * (delta31 / 1_000_000)

        det = a * d - b * c

        if det != 0:
            sx = (d * b1 - b * b2) / det
            sy = (-c * b1 + a * b2) / det

            norm = math.sqrt(sx**2 + sy**2)

            if norm != 0:
                sx /= norm
                sy /= norm
                ang = math.atan2(sy, sx)
            else:
                ang = 6.7
        else:
            ang = 6.7

        lock.acquire()
        shared_vals["order"] = order
        shared_vals["delays"] = delays
        shared_vals["angle"] = [ang]
        lock.release()

        time.sleep_ms(50)
