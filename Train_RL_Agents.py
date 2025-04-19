from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import BaseCallback
import os
from RL_Agent_Env import RL_Env

import torch
import torch.nn as nn
print("CUDA available:", torch.cuda.is_available())


# Normalize observations and rewards
def make_env(rank, env_param_kwargs):
    def _init():
        env = RL_Env(render_mode="fast" if rank==0 else "fast", gui=rank==0, **env_param_kwargs)
        env = Monitor(env)
        return env
    return _init

def adaptive_learning_rate(lr, kl, target_kl=0.01, alpha=1.5):
    """Adjusts learning rate based on KL divergence."""
    if kl < target_kl / 1.5:
        lr *= alpha  # Increase learning rate if KL is too small
    elif kl > target_kl * 1.5:
        lr /= alpha  # Decrease learning rate if KL is too large
    return max(lr, 1e-6)  # Prevent learning rate from becoming too small

class AdaptiveLRCallback(BaseCallback):
    """Custom callback to adjust learning rate based on KL divergence."""
    
    def __init__(self, optimizer, verbose=0):
        super().__init__(verbose)
        self.optimizer = optimizer  # Get optimizer from model

    def _on_step(self):
        """Modify learning rate based on KL divergence."""
        if "approx_kl" in self.locals:
            kl = self.locals["approx_kl"]  # Extract KL divergence from PPO
            new_lr = adaptive_learning_rate(self.optimizer.param_groups[0]['lr'], kl)
            self.optimizer.param_groups[0]['lr'] = new_lr  # Update learning rate
            if self.verbose > 0:
                print(f"KL: {kl:.6f}, Adjusted LR: {new_lr:.6e}")
        return True  # Continue training


if __name__ == "__main__":

    show_last_results = True

    #tr_model_replay_buffer_dir = r"expert_trajectories\recorded_data_sprinting_v1"


    alg_name = "SAC"
    ModellName = "Disney_Imitation_v14"
    main_dir = r"Models\\" + alg_name + ModellName
    models_dir = os.path.join(main_dir, "models")
    logdir = models_dir+"\\logs"

    if not os.path.exists(models_dir):
        os.makedirs(models_dir)
    if not os.path.exists(logdir):
        os.makedirs(logdir)
    
    total_timesteps = 25_000_000  # Increase total training steps

    env_param_kwargs = {
        "ModelType": alg_name,
        "RobotType": "Rabbit_v3",
        "rewards_type": ["Disney_Imitation"],
        "observation_type_stacked": ["head_orientation", "head_linear_acceleration", "joint_torques"],
        "observation_type_solo": ["phase_signal", "last_action", "User_command"],
        "observation_noise": 0.001,
        "Horizon_Length": True,
        "obs_time_space": 1.5,
        "simulation_Timestep": 0.2,
        "terrain_type": "flat",
        "different_terrain": False,
        "different_gravity": True,
        "recorded_movement_file_path_dic": {
                                                #"Expert0_0_v7": 1,
                                                "Expert0_8_v1": 5,
                                                "Expert0_8_v2": 5,
                                                "Expert0_8_v3": 5,
                                                "Expert0_8_v4": 3,
                                                "Expert0_8_v5": 3,
                                                "Expert0_8_v6": 3,

                                             },
        "recorded_movement_file_settings": {
                                                #"Expert0_0_v7": [0],
                                                "Expert0_8_v1": [0.8],
                                                "Expert0_8_v2": [0.8],
                                                "Expert0_8_v3": [0.8],
                                                "Expert0_8_v4": [0.8],
                                                "Expert0_8_v5": [0.8],
                                                "Expert0_8_v6": [0.8],
        },
        "reward_weights": {
                    # Imitation
                    "torso_pos": 2.0,
                    "torso_orient": 2.0,
                    "linear_vel_xy": 1.5,
                    "linear_vel_z": 1.0,
                    "angular_vel_xy": 0.5,
                    "angular_vel_z": 0.5,
                    "LegJoint_pos": 3.0,
                    "LegJoint_vel": 0.5,
                    "component_coordinates_world": 2.0,
                    #"Contact": 1.0,

                    # Regularization
                    "Joint_torques": 0.15,
                    "Joint_acc": 0.15,
                    "action_rate": 0.5,
                    "action_acc": 0.05,

                    # Survival
                    "survival": 1.0,
                }
    }

    if alg_name == "SAC":
        # the episode length is 110 steps
        # 
        hyper_params = {
            "learning_rate": 3*1e-5,#2*1e-5=0.00002
            "batch_size": 128*27, #8192*24,
            "buffer_size": 800_000,
            "policy_kwargs": dict(net_arch=dict(pi=[512, 256], qf=[512, 256])),
            "gamma" : 0.97#0.95#typically in range of 0.8 to 0.99

        }

    elif alg_name == "PPO":
        hyper_params = {
            "learning_rate":2.5e-4,  # The paper mentions using an adaptive learning rate
            "n_steps": 8192 * 12,  # Batch size (envs × steps) in the paper
            "batch_size": 8192,  # From Table IV in the paper
            "ent_coef": 0.0,  # Entropy coefficient
            "gamma": 0.99,  # Discount factor
            "gae_lambda": 0.95,  # GAE discount factor
            "max_grad_norm": 1.0,  # Max gradient norm
            "clip_range": 0.2,  # Clip range
            "n_epochs": 5,  # Number of epochs
            "policy_kwargs": dict(net_arch=[512, 512, 512], activation_fn=nn.ELU)  # Three fully connected layers with ELU activation
        }
    else:
        raise ValueError("Invalid algorithm name")


    if not show_last_results:

        # Make multiprocess env
        num_cpu = 27 # Adjust the number of CPUs based on your machine
        envs = SubprocVecEnv([make_env(i, env_param_kwargs=env_param_kwargs) for i in range(num_cpu)])

        #creates a documentation of the model hyperparameter, the environements parameter and other information concearned to the training, in the Model directory.
        with open(main_dir + "/info.txt", "w") as f:
            f.write(f"Model: {alg_name}\n")
            f.write(f"Model Name: {ModellName}\n")
            f.write(f"Number of CPUs: {num_cpu}\n")
            f.write(f"Total Timesteps: {total_timesteps}\n")
            f.write(f"Env Parameters: {env_param_kwargs}\n")
            f.write(f"Hyperparameters: {hyper_params}\n")

            f.write(f"Used_env_class_filename: {"RL_Agent_Env"}\n")
            #f.write(f"Added rollouts: {tr_model_replay_buffer_dir}\n")
        

        # Define callbacks
        eval_env = RL_Env(render_mode="fast", gui=False, **env_param_kwargs)
        eval_env = Monitor(eval_env)  # Wrap evaluation environment with Monitor

        eval_callback = EvalCallback(eval_env, best_model_save_path=models_dir,
                        log_path=logdir, eval_freq=50000,
                        deterministic=False, render=False,
                        n_eval_episodes=5
                        )

        checkpoint_callback = CheckpointCallback(save_freq=50000, save_path=models_dir,
                            name_prefix='rl_model')
        # Create the custom callback
        #baby_mode_freedom_callback = BabyModeFreedomCallback()

        # # check if there are some replay_buffer data to the trained model
        # if tr_model_replay_buffer_dir != "":
        #     #load the rollouts into the model
        #     #save the expert trajectories
        #     exp_rollouts = serialize.load(tr_model_replay_buffer_dir)
        #     print("Rollouts are added to the replay buffer")
        envs.reset()
        if alg_name == "SAC":
            from stable_baselines3 import SAC
            # #load pre-trained PPO policy
            model = SAC("MlpPolicy", envs,
                        tensorboard_log=logdir,
                        device='cuda',
                        verbose=1,
                        # rollout_buffer_class=startReplayBuffer,
                        # rollout_buffer_kwargs=dict(demonstrations=exp_rollouts, demonstrations_wight=13),
                        **hyper_params
                        )
            #load trained SAC policy
            #model = SAC.load(r"Models\SACDisney_Imitation_v11\models\rl_model_1350000_steps.zip", env=envs, tensorboard_log=logdir)

        elif alg_name == "PPO":
            from stable_baselines3 import PPO
            model = PPO("MlpPolicy", envs,
                        tensorboard_log=logdir,
                        device='cuda',
                        verbose=1,
                        **hyper_params
                        )

        

        model.learn(total_timesteps=total_timesteps, reset_num_timesteps=False,
                    tb_log_name=alg_name, callback=[checkpoint_callback]) #AdaptiveLRCallback(model.policy.optimizer)])

        model.save(f"{models_dir}/final_model")
        model.save_replay_buffer(os.path.join(models_dir, f"rl_model_replay_buffer_{total_timesteps}_steps.pkl"))

        envs.close()
    
    else:
        # Load the trained agent
        model = SAC.load(r"Models\SACDisney_Imitation_v14\models\rl_model_2700000_steps.zip")

        # Evaluate the agent
        env = RL_Env(render_mode="human", gui=True, **env_param_kwargs)
        obs, _ = env.reset()
        for i in range(1000):
            action, info = model.predict(obs)
            obs, rewards, terminated, truncated, info = env.step(action)
            if truncated or terminated:
                obs, _ = env.reset()

        env.close()
