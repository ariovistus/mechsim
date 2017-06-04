# validated: 2015-12-24 DS 6d854af shared/java/edu/wpi/first/wpilibj/Timer.java
#----------------------------------------------------------------------------
# Copyright (c) FIRST 2008-2012. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.
#----------------------------------------------------------------------------

__all__ = ["Timer"]

class Timer:

    def __init__(self):
        self.accumulatedTime = 0.0
        self.time_seconds = 0.0
        self.startTime = self.getMsClock()
        self.running = False

    def getMsClock(self):
        return self.time_seconds * 1000

    def get(self):
        if self.running:
            return ((self.getMsClock() - self.startTime) + self.accumulatedTime) / 1000.0
        else:
            return self.accumulatedTime

    def reset(self):
        self.accumulatedTime = 0.0
        self.startTime = self.getMsClock()

    def start(self):
        self.startTime = self.getMsClock()
        self.running = True

