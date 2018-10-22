
class TrapezoidalProfile:
    def __init__(self, cruise_v: float, a: float, target_pos: float, tolerance: float, current_target_v=0):
        self.cruise_v = cruise_v
        self.a = a
        self.current_target_v = current_target_v
        self.current_a = 0
        self.target_pos = target_pos
        self.tolerance = tolerance

    def calculate_new_velocity(self, current_pos, dt):
        a = self.a
        v = self.current_target_v
        okerr = self.tolerance

        if a == 0:
            adist = v * 100 * self.tolerance
        else:
            adist = 0.5 * v ** 2 / a

        err = self.target_pos - current_pos

        self.current_a = 0

        def inc():
            nonlocal v, dt, a
            if v + dt * a  > self.cruise_v:
                self.current_a = (self.cruise_v - v) / dt
                v = self.cruise_v
            else:
                v += dt * a
                self.current_a = +a

        def dec():
            nonlocal v, dt, a
            if v - dt * a  < -self.cruise_v:
                self.current_a = (self.cruise_v - v) / dt
                v = -self.cruise_v
            else:
                v -= dt * a
                self.current_a = -a

        if abs(err) < okerr:
            if v > 0:
                v -= min(v, dt * a)
            elif v < 0:
                v -= max(v, -dt * a)
        elif okerr <= err < adist and v > 0:
            dec()
        elif okerr <= err < adist and v < 0:
            inc()
        elif -okerr >= err > -adist and v < 0:
            inc()
        elif -okerr >= err > -adist and v > 0:
            dec()
        elif err > adist and v >= 0:
            if v < self.cruise_v:
                inc()
            elif v > self.cruise_v:
                dec()
        elif err < -adist and v <= 0:
            if v > -self.cruise_v:
                dec()
            elif v < -self.cruise_v:
                inc()
        elif err > adist and v < 0:
            inc()
        elif err < -adist and v > 0:
            dec()

        self.current_target_v = v
