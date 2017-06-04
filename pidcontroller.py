from collections import deque
from wpilib import (
    PIDController,
)
from timer import Timer
from wpilib.interfaces import PIDSource
from wpilib._impl.utils import (
    match_arglist, 
    HasAttribute,
)


class NoThreadingPIDController(PIDController):
    def __init__(self, Kp, Ki, Kd, *args, **kwargs):
        f_arg = ("Kf", [float, int])
        source_arg = ("source", [HasAttribute("pidGet"), HasAttribute("__call__")])
        output_arg = ("output", [HasAttribute("pidWrite"), HasAttribute("__call__")])
        period_arg = ("period", [float, int])

        templates = [[f_arg, source_arg, output_arg, period_arg],
                     [source_arg, output_arg, period_arg],
                     [source_arg, output_arg],
                     [f_arg, source_arg, output_arg]]

        _, results = match_arglist('PIDController.__init__',
                                   args, kwargs, templates)

        self.P = Kp  # factor for "proportional" control
        self.I = Ki  # factor for "integral" control
        self.D = Kd  # factor for "derivative" control
        self.F = results.pop("Kf", 0.0)  # factor for feedforward term
        self.pidOutput = results.pop("output")
        self.pidInput = results.pop("source")
        self.period = results.pop("period", self.kDefaultPeriod)
        
        self.pidInput = PIDSource.from_obj_or_callable(self.pidInput)
        
        if hasattr(self.pidOutput, 'pidWrite'):
            self.pidOutput = self.pidOutput.pidWrite

        self.maximumOutput = 1.0    # |maximum output|
        self.minimumOutput = -1.0   # |minimum output|
        self.maximumInput = 0.0     # maximum input - limit setpoint to this
        self.minimumInput = 0.0     # minimum input - limit setpoint to this
        self.continuous = False     # do the endpoints wrap around? eg. Absolute encoder
        self.enabled = False        # is the pid controller enabled
        self.prevError = 0.0        # the prior error (used to compute velocity)
        self.totalError = 0.0       #the sum of the errors for use in the integral calc
        self.buf = deque(maxlen=1)
        self.setpoint = 0.0
        self.prevSetpoint = 0.0
        self.error = 0.0
        self.result = 0.0
        self.mutex = WithStub()
        self.setpointTimer = Timer()
        self.setpointTimer.start()


class WithStub:
    def __enter__(self):
        pass

    def __exit__(self, *args, **kwargs):
        pass


if __name__ == '__main__':
    def source():
        return 2.3

    def output(x):
        print ('output: ', x)

    pid = NoThreadingPIDController(Kp = 1.0, Ki = 0.3, Kd = 0.4, source = source, output = output, period = 50)
    pid.setInputRange(0.1, 5.1)
    pid.setOutputRange(0, 4)
    pid.setSetpoint( 3.1)
    pid.enable()
    for i in range(10):
        pid._calculate()
