from Trajectory import TrajectoryRecorder

import matplotlib.pyplot as plt

Trajecotry_Base = TrajectoryRecorder()
Trajecotry_Base.load_trajectory("Bewegung2")
keys = Trajecotry_Base.get_keys()
values = Trajecotry_Base.get_values()
times = Trajecotry_Base.get_times()

