import random
import heapq
from collections import defaultdict

class FlashTrajectoryDeck:
    """
    Flashcard-style trajectory generator focusing on adaptive learning.
    
    Features:
    - Poorly performing trajectories appear more often.
    - Every fourth selection is fully randomized.
    - A trajectory cannot be selected more than `max_revisions` times before another is prioritized.

    Args:
        trajectory_dic (dict): Dictionary of trajectories with difficulty levels (1-5).
        max_revisions (int, optional): Maximum times a trajectory can be chosen before forcing another. Default is 10.
        random_interval (int, optional): Every `random_interval` turns, a random trajectory is selected. Default is 4.
    """

    def __init__(self, trajectory_dic: dict, max_revisions: int = 10, random_interval: int = 4):
        self.trajectory_dic = trajectory_dic  # { "path1": 3, "path2": 5, ... }
        self.trajectory_counts = defaultdict(int)  # Counts how often a trajectory has been shown
        self.trajectory_feedback = {t: 1.0 for t in trajectory_dic}  # Feedback score (1 = neutral start)
        self.trajectory_queue = []  # Priority queue for scheduling
        self.max_revisions = max_revisions
        self.random_interval = random_interval
        self.call_count = 0  # Track the number of calls for randomization

        #for one episode feedback
        self.episode_feedback = []

        # Initialize priority queue
        for trajectory, difficulty in trajectory_dic.items():
            priority = self.calculate_priority(0, difficulty, 1.0)
            heapq.heappush(self.trajectory_queue, (priority, trajectory))

    def calculate_priority(self, count: int, difficulty: int, feedback_factor: float) -> float:
        """
        Computes the priority score for a trajectory.
        Lower scores mean higher priority.
        """
        base_priority = count
        difficulty_weight = 1.0 / difficulty  # Harder trajectories appear more often
        adjusted_priority = (base_priority + 1) * difficulty_weight / feedback_factor
        return adjusted_priority

    def get_trajectory_path(self) -> str:
        """
        Returns the next trajectory based on adaptive scheduling.
        """
        self.call_count += 1

        # Random selection every `random_interval` calls
        if self.call_count % self.random_interval == 0:
            return random.choice(list(self.trajectory_dic.keys()))

        # Retrieve the best candidate from the queue
        while self.trajectory_queue:
            priority, candidate = heapq.heappop(self.trajectory_queue)

            # Enforce max revisions rule
            if self.trajectory_counts[candidate] >= self.max_revisions:
                continue  # Skip this candidate and try another

            # Select this candidate
            self.trajectory_counts[candidate] += 1
            return candidate

        # If queue is empty, pick a random trajectory
        return random.choice(list(self.trajectory_dic.keys()))
    
    def collect_episode_feedback(self, score_percent: float):
        """
        Updates feedback for a trajectory (score in range 0.0 - 1.0).
        """
        # Avoid division by zero (ensure feedback is in range)
        score_percent = max(0.01, min(1.0, score_percent))
        self.episode_feedback.append(score_percent)

    def return_episode_feedback(self):
        """
        Returns the feedback for the episode.
        """
        num_steps = len(self.episode_feedback)
        if num_steps != 0:
            avg_score = sum(self.episode_feedback) / num_steps
            self.episode_feedback = []
            return avg_score
        return 0.0

    def give_feedback(self, trajectory: str, score_percent: float):
        """
        Updates feedback for a trajectory (score in range 0.0 - 1.0).
        """
        if trajectory not in self.trajectory_dic:
            print(f"Warning: {trajectory} not found.")
            return

        # Avoid division by zero (ensure feedback is in range)
        score_percent = max(0.01, min(1.0, score_percent))

        # Update feedback factor
        self.trajectory_feedback[trajectory] = score_percent

        # Reinsert with updated priority
        difficulty = self.trajectory_dic[trajectory]
        new_priority = self.calculate_priority(self.trajectory_counts[trajectory], difficulty, score_percent)
        heapq.heappush(self.trajectory_queue, (new_priority, trajectory))


# Example usage
if __name__ == "__main__":
    trajectory_list = {
        "path1": 3,  # Medium difficulty
        "path2": 5,  # Hard
        "path3": 2,  # Easy
        "path4": 4   # High-medium difficulty
    }

    deck = FlashTrajectoryDeck(trajectory_list)

    for _ in range(20):
        traj = deck.get_trajectory_path()
        print(f"Next trajectory: {traj}")
        score = float(input("Enter score (0-1): "))  # Get user feedback
        deck.give_feedback(traj, score)
