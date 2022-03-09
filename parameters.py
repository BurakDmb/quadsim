from src.envs.quad import DeterministicQuad, linear_quad_dynamics
import numpy as np


def createEnv(eval=False):
    if not eval:
        env = DeterministicQuad(linear_quad_dynamics,
                                t_end=t_end,
                                simulation_freq=simulation_freq,
                                control_freq=control_freq,
                                keep_history=keep_history)
    else:
        constant_reference = np.array([1, 0, 1, 0, 1, 0])
        env = DeterministicQuad(linear_quad_dynamics,
                                t_end=t_end,
                                simulation_freq=simulation_freq,
                                control_freq=control_freq,
                                keep_history=keep_history,
                                set_constant_reference=True,
                                constant_reference=constant_reference,
                                eval_env=True)
    return env


total_timesteps = 2_000_000
t_end = 5
simulation_freq = 250
control_freq = 50
keep_history = False

number_of_parallel_experiments = 1


# Change the flags to True/False for only running specific agents
start_ddpg = True
start_ppo = True
start_sac = True
start_td3 = True
start_a2c = True

ddpg_learning_setting = {}
ddpg_learning_setting['env'] = createEnv()
ddpg_learning_setting['env_eval'] = createEnv(eval=True)
ddpg_learning_setting['learning_rate'] = 1e-3
ddpg_learning_setting['discount_rate'] = 0.99
ddpg_learning_setting['nn_layer_size'] = 64
ddpg_learning_setting['batch_size'] = 100
ddpg_learning_setting['tb_log_name'] = "ddpg-quad"
ddpg_learning_setting['tb_log_dir'] = "./logs/quad_tensorboard/"
ddpg_learning_setting['total_timesteps'] = total_timesteps
ddpg_learning_setting['seed'] = None
ddpg_learning_setting['policy'] = "MlpPolicy"
ddpg_learning_setting['save'] = True
ddpg_learning_setting['device'] = 'cpu'

ppo_learning_setting = {}
ppo_learning_setting['env'] = createEnv()
ppo_learning_setting['env_eval'] = createEnv(eval=True)
ppo_learning_setting['learning_rate'] = 3e-4
ppo_learning_setting['discount_rate'] = 0.99
ppo_learning_setting['nn_layer_size'] = 64
ppo_learning_setting['n_steps'] = 2048
ppo_learning_setting['tb_log_name'] = "ppo-quad"
ppo_learning_setting['tb_log_dir'] = "./logs/quad_tensorboard/"
ppo_learning_setting['total_timesteps'] = total_timesteps
ppo_learning_setting['seed'] = None
ppo_learning_setting['policy'] = "MlpPolicy"
ppo_learning_setting['save'] = True
ppo_learning_setting['device'] = 'cpu'

sac_learning_setting = {}
sac_learning_setting['env'] = createEnv()
sac_learning_setting['env_eval'] = createEnv(eval=True)
sac_learning_setting['learning_rate'] = 3e-4
sac_learning_setting['discount_rate'] = 0.99
sac_learning_setting['nn_layer_size'] = 64
sac_learning_setting['batch_size'] = 100
sac_learning_setting['tb_log_name'] = "sac-quad"
sac_learning_setting['tb_log_dir'] = "./logs/quad_tensorboard/"
sac_learning_setting['total_timesteps'] = total_timesteps
sac_learning_setting['seed'] = None
sac_learning_setting['policy'] = "MlpPolicy"
sac_learning_setting['save'] = True
sac_learning_setting['device'] = 'cpu'

td3_learning_setting = {}
td3_learning_setting['env'] = createEnv()
td3_learning_setting['env_eval'] = createEnv(eval=True)
td3_learning_setting['learning_rate'] = 3e-4
td3_learning_setting['discount_rate'] = 0.99
td3_learning_setting['nn_layer_size'] = 64
td3_learning_setting['batch_size'] = 100
td3_learning_setting['tb_log_name'] = "td3-quad"
td3_learning_setting['tb_log_dir'] = "./logs/quad_tensorboard/"
td3_learning_setting['total_timesteps'] = total_timesteps
td3_learning_setting['seed'] = None
td3_learning_setting['policy'] = "MlpPolicy"
td3_learning_setting['save'] = True
td3_learning_setting['device'] = 'cpu'

a2c_learning_setting = {}
a2c_learning_setting['env'] = createEnv()
a2c_learning_setting['env_eval'] = createEnv(eval=True)
a2c_learning_setting['learning_rate'] = 3e-4
a2c_learning_setting['discount_rate'] = 0.99
a2c_learning_setting['nn_layer_size'] = 64
a2c_learning_setting['n_steps'] = 5
a2c_learning_setting['tb_log_name'] = "a2c-quad"
a2c_learning_setting['tb_log_dir'] = "./logs/quad_tensorboard/"
a2c_learning_setting['total_timesteps'] = total_timesteps
a2c_learning_setting['seed'] = None
a2c_learning_setting['policy'] = "MlpPolicy"
a2c_learning_setting['save'] = True
a2c_learning_setting['device'] = 'cpu'
