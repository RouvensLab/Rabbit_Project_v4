import json
import os


class TrajectoryRecorder:
    def __init__(self):
        pass
        

        

    def new_trajectory(self, name, data_structure:list, path=r"Trajectories\Recordings"):
        """Creates a new trajectory file with the given name and data structure.
        name: str
        data_structure: list with the real data names e.g. ["joint_angles", "joint_velocities", "joint_torques"]
        
        """
        self.trajectory = []
        self.trajectory_times = []
        self.data_structure = data_structure
        self.main_dir = path
        self.trajectory_name = name
        self.trajectory_path = os.path.join(self.main_dir, self.trajectory_name + ".json")
        #check if the directory exists
        if not os.path.exists(self.main_dir):
            os.makedirs(self.main_dir)
        #check if the file exists
        if not os.path.exists(self.trajectory_path):
            with open(self.trajectory_path, "w") as f:
                f.write("[]")
        else:
            print("Trajectory already exists")
            #overwrite the file
            with open(self.trajectory_path, "w") as f:
                f.write("[]")
        
    def load_trajectory(self, name, path=r"Trajectories\Recordings"):
        self.trajectory_name = name
        self.main_dir = path
        self.trajectory_path = os.path.join(self.main_dir, self.trajectory_name + ".json")

        with open(self.trajectory_path, "r") as f:
            data = json.load(f)
            self.data_structure = data["data_structure"]
            self.trajectory = data["trajectory"]
            self.trajectory_times = data["time"]

    def get_keys(self):
        return self.data_structure
    def get_times(self):
        return self.trajectory_times
    def get_values(self):
        return self.trajectory
    
    def get_keyValues(self, key):
        index = self.data_structure.index(key)
        values = [d[index] for d in self.trajectory]
        return values
    

    def convert_data_to_json_format(self, data):
        """
        Convert the data to a json serializable format
        Sometimes the data contains numpy arrays or other data types that are not serializable

        """
        new_data = []
        for d in data:
            if isinstance(d, (list, float, int, str)):
                new_data.append(d)
            elif isinstance(d, tuple):
                new_data.append(list(d))
            else:
                #for numpy arrays
                new_data.append(d.tolist())
        return new_data
        
        
    def add_data(self, data, time):
        """Add data to the trajectory"""
        print(f"Add Data {data}")
        #convert data to a json serializable format
        self.trajectory.append(self.convert_data_to_json_format(data))
        self.trajectory_times.append(time)

    def get_near_data(self, time):
        """Get the data that is closest to the given time"""
        if len(self.trajectory_times) == 0:
            return self.trajectory[0]
        #get the index of the closest time
        index = min(range(len(self.trajectory_times)), key=lambda i: abs(self.trajectory_times[i]-time))
        return self.trajectory[index]

    def save_trajectory(self):
        #print(f"Save Trajectory {self.trajectory_name}")
        #print(self.trajectory)
        with open(self.trajectory_path, "w") as f:
            json.dump({"data_structure": self.data_structure, "trajectory": self.trajectory, "time": self.trajectory_times}, f)

    def get_trajectory_infos(self):
        #steps
        steps = len(self.trajectory)
        #time
        time = self.trajectory_times[-1]
        return steps, time
    

if __name__=="__main__":
    trajRec = TrajectoryRecorder()
    trajRec.load_trajectory("PushSprint_v1")
    print(trajRec.get_trajectory_infos())
        
