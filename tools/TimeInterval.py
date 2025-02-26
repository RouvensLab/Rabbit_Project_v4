import time

class TimeInterval:
    def __init__(self, time_per_step):
        self.time_per_step = time_per_step
        self.last_time = time.time()

    def wait_for_step(self, no_standart_timeStep = None):
        if no_standart_timeStep == None:
            no_standart_timeStep = self.time_per_step
        sleeping_time = no_standart_timeStep - (time.time() - self.last_time)
        if sleeping_time > 0:
            time.sleep(sleeping_time)
        else:
            print("Warning: the loop took longer than the specified time per step.")
        self.last_time = time.time()

# Example usage:
# interval = TimeInterval(2)  # 2 seconds per step
# interval.wait_for_step()  # This will wait for 2 seconds