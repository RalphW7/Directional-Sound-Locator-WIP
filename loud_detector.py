from machine import ADC
import time
import _thread
import math

adc1 = ADC(26)
adc2 = ADC(27)
adc3 = ADC(28)

THRESHOLD = 40000      # adjust experimentally
WAIT_WINDOW_US = 3000  # how long to wait for other mics

shared_vals = {
    "order": [],
    "delays": []
}

lock = _thread.allocate_lock()


def print_thread():
    while True:
        lock.acquire()
        order = shared_vals["order"]
        delays = shared_vals["delays"]
        lock.release()

        if order:
            print("Detection order:", order)
            print("Delays (us):", delays)

        time.sleep(0.1)


_thread.start_new_thread(print_thread, ())

#mic positions:
#     3
# 1   2

# mic positions in cm grid. mic treated as origin
pos1 = [0,0]
pos2 = [7.7, 0]
pos3 = [7.7, 5.6]

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

        # compute order and delays
        detections = []
        for i, t in enumerate(times):
            if t is not None:
                detections.append((i + 1, t))

        detections.sort(key=lambda x: x[1])

        if detections:

            first_time = detections[0][1]

            order = []
            delays = []

            for mic, t in detections:
                order.append(mic)
                delays.append(time.ticks_diff(t, first_time))

            lock.acquire()
            shared_vals["order"] = order
            shared_vals["delays"] = delays
            lock.release()

        # small debounce so same sound doesn't retrigger
        time.sleep_ms(50)