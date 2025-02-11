import random
import heapq

class FlashTrajectoryDeck:
    """
    Flashcard-style trajectory generator using spaced repetition, user feedback, and difficulty-based scheduling.

    Features:
    - Spaced repetition: Frequently used trajectories appear less often.
    - Adaptive learning: Poorly imitated trajectories appear more often.
    - Difficulty-based scheduling: Harder trajectories appear more often.
    - Priority queue scheduling: Lower score = higher priority.
    - Allows setting a start trajectory.

    Args:
        trajectory_list (dict): Dictionary with trajectory names as keys and difficulty (1-5) as values.
        start_trajectory (str, optional): A specific trajectory to start with.
    """

    def __init__(self, trajectory_list, start_trajectory=None):
        """
        Initializes the trajectory deck.

        Args:
            trajectory_list (dict): Dictionary {trajectory_name: difficulty (1-5)}.
            start_trajectory (str, optional): A specific trajectory to start with.
        """
        self.trajectory_list = trajectory_list  # { "path1": 3, "path2": 5, ... }
        self.trajectory_counts = {t: 0 for t in trajectory_list}  # Tracks how often each trajectory is shown
        self.trajectory_feedback = {t: 1.0 for t in trajectory_list}  # Tracks feedback (higher = better performance)
        self.trajectory_queue = []  # Priority queue for selecting trajectories

        # Initialize priority queue with difficulty-based weighting
        for trajectory, difficulty in trajectory_list.items():
            weighted_priority = self.calculate_priority(0, difficulty, 1.0)
            heapq.heappush(self.trajectory_queue, (weighted_priority, trajectory))

        # Set the starting trajectory, if provided
        self.start_trajectory = start_trajectory
        self.first_call = True  # Ensures start trajectory is used first

    def calculate_priority(self, count, difficulty, feedback_factor):
        """
        Computes the priority score for a trajectory.

        Lower scores = higher priority.

        Args:
            count (int): Number of times trajectory has been shown.
            difficulty (int): Difficulty level (1-5).
            feedback_factor (float): Performance feedback factor.

        Returns:
            float: Computed priority score.
        """
        # Base priority is determined by the count
        base_priority = count

        # More difficult trajectories have a base priority reduction (shown more often)
        difficulty_weight = 1.0 / difficulty  # Harder trajectories get lower priority scores (higher frequency)

        # Feedback factor modifies priority (higher feedback = more delay)
        adjusted_priority = (base_priority + 1) * difficulty_weight * feedback_factor  

        return adjusted_priority

    def get_trajectory_path(self):
        """
        Returns the next trajectory based on spaced repetition and difficulty.

        Returns:
            str: Selected trajectory path.
        """

        # If it's the first call and a start trajectory is defined
        if self.first_call and self.start_trajectory:
            self.first_call = False
            return self.start_trajectory

        # Get the trajectory with the lowest priority (most urgent for review)
        _, trajectory = heapq.heappop(self.trajectory_queue)

        # Update trajectory usage count
        self.trajectory_counts[trajectory] += 1

        return trajectory

    def give_feedback(self, trajectory, score):
        """
        Receives user feedback on how well the trajectory was imitated.

        Args:
            trajectory (str): The trajectory that was just executed.
            score (int): Imitation score (1 = poor, 5 = perfect).
        """

        if trajectory not in self.trajectory_counts:
            print(f"Warning: {trajectory} is not in the trajectory list.")
            return

        # Convert score to a weight factor (higher score = more delay)
        # Example: Score 1 (bad) → factor 0.5 (comes back sooner)
        #          Score 5 (good) → factor 2.0 (delays next appearance)
        feedback_factor_map = {1: 0.5, 2: 0.75, 3: 1.0, 4: 1.5, 5: 2.0}
        feedback_factor = feedback_factor_map.get(score, 1.0)

        # Update trajectory feedback score
        self.trajectory_feedback[trajectory] = feedback_factor

        # Compute new priority with difficulty weighting
        difficulty = self.trajectory_list[trajectory]
        new_priority = self.calculate_priority(self.trajectory_counts[trajectory], difficulty, feedback_factor)

        # Reinsert into the queue with updated priority
        heapq.heappush(self.trajectory_queue, (new_priority, trajectory))


if __name__ == "__main__":
    # Example Usage:
    trajectory_list = {
        "path1": 3,  # Medium difficulty
        "path2": 5,  # Hard
        "path3": 2,  # Easy
        "path4": 4   # High-medium difficulty
    }
    deck = FlashTrajectoryDeck(trajectory_list, start_trajectory="path3")

    # Get and evaluate trajectories
    for _ in range(10):
        traj = deck.get_trajectory_path()
        print(f"Next trajectory: {traj}")

        # Simulating user feedback (random scores for testing)
        score = random.randint(1, 5)
        print(f"User feedback for {traj}: {score}")
        deck.give_feedback(traj, score)