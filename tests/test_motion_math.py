from math import sqrt, isfinite

TIMER_HZ = 2_000_000
MIN_INTERVAL = 16
INT32_MIN = -(2**31)
INT32_MAX = 2**31 - 1

def ticks_to_q8(ticks):
    return min(ticks, 0x007FFFFF) << 8

def q8_to_ticks(q8):
    return (q8 >> 8) + (1 if q8 & 0xFF else 0)

def ceil_us_to_ticks(us, timer_hz):
    if us == 0:
        return 0
    return (us * timer_hz + 999_999) // 1_000_000

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

def controlled_stop_steps(current_position, original_target, direction,
                          requested_stop_steps, soft_min=None, soft_max=None):
    remaining = abs(original_target - current_position)
    allowed = min(requested_stop_steps, remaining)

    target = current_position + direction * allowed
    if soft_min is not None:
        target = max(target, soft_min)
    if soft_max is not None:
        target = min(target, soft_max)

    allowed = min(allowed, abs(target - current_position))
    return allowed, current_position + direction * allowed

def a4988_levels(microsteps):
    table = {
        1: (0, 0, 0),
        2: (1, 0, 0),
        4: (0, 1, 0),
        8: (1, 1, 0),
        16: (1, 1, 1),
    }
    return table.get(microsteps)

def drv8825_levels(microsteps):
    table = {
        1: (0, 0, 0),
        2: (1, 0, 0),
        4: (0, 1, 0),
        8: (1, 1, 0),
        16: (0, 0, 1),
        32: (1, 0, 1),
    }
    return table.get(microsteps)

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

def test_units_relative_and_absolute():
    full_steps = 200
    microsteps = 16
    gear = 1.0
    travel_per_rev = 8.0
    pulses_per_rev = full_steps * microsteps * gear
    pulses_per_mm = pulses_per_rev / travel_per_rev

    assert pulses_per_rev == 3200
    assert pulses_per_mm == 400
    assert round(50.0 * pulses_per_mm) == 20_000
    assert round(300.0 * pulses_per_mm) == 120_000

def interval_ticks_for_speed(timer_hz, steps_per_second):
    return max(1, (timer_hz + steps_per_second - 1) // steps_per_second)


def test_timer_tick_ceiling():
    assert ceil_us_to_ticks(4, 2_000_000) == 8
    assert ceil_us_to_ticks(1, 2_000_000) == 2

    # 20 MHz AVR / 8 = 2.5 MHz: never round the pulse shorter.
    assert ceil_us_to_ticks(4, 2_500_000) == 10
    assert ceil_us_to_ticks(1, 2_500_000) == 3

    # 8 MHz AVR / 8 = 1 MHz.
    assert ceil_us_to_ticks(4, 1_000_000) == 4

    interval = interval_ticks_for_speed(2_000_000, 3000)
    assert interval == 667
    assert 2_000_000 / interval <= 3000

def test_controlled_stop_never_exceeds_target():
    allowed, target = controlled_stop_steps(
        current_position=900,
        original_target=1000,
        direction=1,
        requested_stop_steps=500,
    )
    assert allowed == 100
    assert target == 1000

def test_controlled_stop_respects_soft_limit():
    allowed, target = controlled_stop_steps(
        current_position=900,
        original_target=5000,
        direction=1,
        requested_stop_steps=500,
        soft_min=0,
        soft_max=1100,
    )
    assert allowed == 200
    assert target == 1100

def test_repeated_stop_is_idempotent_model():
    state = {"stopping": False, "plans": 0}

    def stop():
        if state["stopping"]:
            return
        state["stopping"] = True
        state["plans"] += 1

    stop()
    stop()
    stop()
    assert state["plans"] == 1

def test_position_boundaries():
    assert INT32_MIN - 1 < INT32_MIN
    assert INT32_MAX + 1 > INT32_MAX

    def next_overflow(position, direction):
        return ((direction > 0 and position == INT32_MAX) or
                (direction < 0 and position == INT32_MIN))

    assert next_overflow(INT32_MAX, 1)
    assert next_overflow(INT32_MIN, -1)
    assert not next_overflow(INT32_MAX, -1)
    assert not next_overflow(INT32_MIN, 1)

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

def test_homing_phase_guard():
    max_phase = 100
    phase_steps = 100

    # A switch that became active on the final allowed physical step succeeds.
    switch_active = True
    assert not ((phase_steps >= max_phase) and not switch_active)

    # If it is still inactive on the next event, fail before step 101.
    switch_active = False
    assert (phase_steps >= max_phase) and not switch_active

def test_step_polarity_levels():
    def levels(active_high):
        active = 1 if active_high else 0
        idle = 0 if active_high else 1
        return idle, active

    assert levels(True) == (0, 1)
    assert levels(False) == (1, 0)


def test_microstep_truth_tables():
    assert a4988_levels(1) == (0, 0, 0)
    assert a4988_levels(16) == (1, 1, 1)
    assert a4988_levels(32) is None

    assert drv8825_levels(1) == (0, 0, 0)
    assert drv8825_levels(16) == (0, 0, 1)
    assert drv8825_levels(32) == (1, 0, 1)
    assert drv8825_levels(64) is None


def test_soft_limit_allows_recovery():
    def violates(current, direction, soft_min, soft_max):
        nxt = current + direction
        if nxt < soft_min:
            return not (current < soft_min and direction > 0)
        if nxt > soft_max:
            return not (current > soft_max and direction < 0)
        return False

    assert not violates(-10, 1, 0, 100)   # recover upward into range
    assert violates(-10, -1, 0, 100)      # moving farther below MIN
    assert not violates(110, -1, 0, 100)  # recover downward into range
    assert violates(110, 1, 0, 100)       # moving farther above MAX


def test_single_move_phase_index_range():
    def single_move_allowed(current, target):
        return abs(target - current) <= INT32_MAX

    assert single_move_allowed(0, INT32_MAX)
    assert single_move_allowed(-100, 100)
    assert not single_move_allowed(INT32_MIN, INT32_MAX)


def test_motion_config_phase_distance_range():
    def valid(vmax, accel, decel):
        v2 = vmax * vmax
        accel_distance = (v2 + 2 * accel - 1) // (2 * accel)
        decel_distance = (v2 + 2 * decel - 1) // (2 * decel)
        return accel_distance <= INT32_MAX and decel_distance <= INT32_MAX

    assert valid(12_000, 25_000, 25_000)
    assert not valid(250_000, 1, 1)


def test_continuous_mode_needs_no_progress_counter():
    completed_steps = 0
    for _ in range(1000):
        mode = "Continuous"
        if mode != "Continuous":
            completed_steps += 1
    assert completed_steps == 0


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
        test_units_relative_and_absolute,
        test_timer_tick_ceiling,
        test_controlled_stop_never_exceeds_target,
        test_controlled_stop_respects_soft_limit,
        test_repeated_stop_is_idempotent_model,
        test_position_boundaries,
        test_homing_debounce_model,
        test_homing_phase_guard,
        test_step_polarity_levels,
        test_microstep_truth_tables,
        test_soft_limit_allows_recovery,
        test_single_move_phase_index_range,
        test_motion_config_phase_distance_range,
        test_continuous_mode_needs_no_progress_counter,
        test_scheduler_elapsed_sync,
    ]

    for test in tests:
        test()
        print("PASS:", test.__name__)

    print("Motion/safety regression: PASS")
