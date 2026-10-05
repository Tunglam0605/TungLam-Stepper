from math import sqrt

TIMER_HZ = 2_000_000
Q = 256
MIN_INTERVAL = 16

def ticks_to_q8(ticks):
    return min(ticks, 0x007FFFFF) << 8

def q8_to_ticks(q8):
    return (q8 >> 8) + (1 if q8 & 0xFF else 0)

def plan(steps, vmax, accel, decel):
    peak2 = float(vmax * vmax)
    accel_steps = int(peak2 / (2.0 * accel))
    decel_steps = int(peak2 / (2.0 * decel))
    peak_speed = float(vmax)

    if accel_steps + decel_steps > steps:
        peak2 = (2.0 * steps * accel * decel) / (accel + decel)
        peak_speed = sqrt(peak2)
        accel_steps = int(peak2 / (2.0 * accel))
        if accel_steps >= steps:
            accel_steps = steps - 1 if steps > 1 else 0
        decel_steps = steps - accel_steps

    cruise_steps = steps - min(steps, accel_steps + decel_steps)
    min_interval = max(MIN_INTERVAL, TIMER_HZ // max(1, int(peak_speed)))
    c0 = int(0.676 * TIMER_HZ * sqrt(2.0 / accel))
    c0 = max(c0, min_interval)

    return accel_steps, cruise_steps, decel_steps, min_interval, c0

def simulate(steps, vmax, accel, decel):
    a, c, d, min_interval, c0 = plan(steps, vmax, accel, decel)
    interval_q8 = ticks_to_q8(c0)
    min_q8 = ticks_to_q8(min_interval)
    accel_n = 0
    decel_n = -max(1, d)
    intervals = []

    for completed in range(1, steps + 1):
        intervals.append(q8_to_ticks(interval_q8))
        if completed >= steps:
            break

        if completed < a:
            accel_n += 1
            denom = 4 * accel_n + 1
            delta = (2 * interval_q8) // denom
            if delta < interval_q8:
                interval_q8 -= delta
            interval_q8 = max(interval_q8, min_q8)
        elif completed < a + c:
            interval_q8 = min_q8
        else:
            if decel_n >= 0:
                decel_n = -max(1, steps - completed)
            denom_signed = 4 * decel_n + 1
            if denom_signed < 0:
                delta = (2 * interval_q8) // (-denom_signed)
                interval_q8 = min(0x7FFFFFFF, interval_q8 + delta)
            decel_n += 1
        interval_q8 = max(interval_q8, min_q8)

    return (a, c, d), intervals

def test_long_trapezoid():
    phases, intervals = simulate(20_000, 8_000, 12_000, 12_000)
    a, c, d = phases
    assert a > 0 and c > 0 and d > 0
    assert min(intervals) >= MIN_INTERVAL
    assert intervals[min(a - 1, len(intervals)-1)] <= intervals[0]
    if c > 2:
        cruise = intervals[a:a + c - 1]
        assert max(cruise) - min(cruise) <= 1
    assert intervals[-1] > min(intervals)

def test_short_triangular():
    phases, intervals = simulate(400, 20_000, 20_000, 20_000)
    a, c, d = phases
    assert c == 0
    assert a + d == 400
    assert intervals[-1] > min(intervals)

def test_q8_prevents_integer_stall():
    _, intervals = simulate(30_000, 12_000, 25_000, 25_000)
    unique = len(set(intervals[:5000]))
    assert unique > 25, unique

def test_units():
    full_steps = 200
    microsteps = 16
    gear = 1.0
    travel_per_rev = 8.0
    pulses_per_rev = full_steps * microsteps * gear
    assert pulses_per_rev == 3200
    assert pulses_per_rev / travel_per_rev == 400

def test_homing_debounce_model():
    required = 3
    active = 0
    triggered_at = None
    samples = [False, True, False, True, True, True]
    for i, state in enumerate(samples):
        active = min(255, active + 1) if state else 0
        if active >= required:
            triggered_at = i
            break
    assert triggered_at == 5

def test_scheduler_elapsed_sync():
    axis1_remaining = 1000
    elapsed = 240
    axis1_remaining = max(0, axis1_remaining - elapsed)
    axis2_first = 500
    assert axis1_remaining == 760
    assert min(axis1_remaining, axis2_first) == 500

if __name__ == "__main__":
    tests = [
        test_long_trapezoid,
        test_short_triangular,
        test_q8_prevents_integer_stall,
        test_units,
        test_homing_debounce_model,
        test_scheduler_elapsed_sync,
    ]
    for test in tests:
        test()
        print("PASS:", test.__name__)
    print("Motion regression: PASS")
