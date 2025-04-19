class PhaseGenerator:
    def __init__(self, is_periodic=True, duration=1.0, dt=0.02):
        """
        Initialize a phase signal generator.

        Args:
            is_periodic (bool): If True, phase loops continuously (for walking).
                                If False, phase stops at 1 (for episodic motions).
            duration (float): Total time for one cycle.
            dt (float): Time step per update.
        """
        self.phase = 0.0
        self.dt = dt
        self.duration = duration
        self.is_periodic = is_periodic

    def update(self):
        """Update the phase signal."""
        if self.duration <= 0:
            return 0  # Avoid division by zero
        self.phase += self.dt / self.duration
        if self.is_periodic:
            self.phase %= 1.0  # Loop phase for walking
        else:
            self.phase = min(self.phase, 1.0)  # Stop at 1 for episodic motions
        return self.phase