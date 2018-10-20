
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

        if abs(err) < okerr:
            if v > 0:
                v -= min(v, dt * a)
            elif v < 0:
                v -= max(v, -dt * a)
        elif okerr <= err < adist and v > 0:
            v -= dt * a
            self.current_a = -a
        elif okerr <= err < adist and v < 0:
            v += dt * a
            self.current_a = +a
        elif -okerr >= err > -adist and v < 0:
            v += dt * a
            self.current_a = +a
        elif -okerr >= err > -adist and v > 0:
            v -= dt * a
            self.current_a = -a
        elif err > adist and v >= 0:
            if v < self.cruise_v:
                v += dt * a
                self.current_a = +a
            elif v > self.cruise_v:
                v -= dt * a
                self.current_a = -a
        elif err < -adist and v <= 0:
            if v > -self.cruise_v:
                v -= dt * a
                self.current_a = -a
            elif v < -self.cruise_v:
                v += dt * a
                self.current_a = +a
        elif err > adist and v < 0:
            v += dt * a
            self.current_a = +a
        elif err < -adist and v > 0:
            v -= dt * a
            self.current_a = -a

        self.current_target_v = v
