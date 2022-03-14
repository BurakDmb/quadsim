from stable_baselines3 import DDPG, PPO, SAC, A2C, TD3
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.logger import TensorBoardOutputFormat
import datetime
import os


def train_ddpg_agent(learning_setting):
    """
    PPO(Proximal Policy Optimization Algorithm, Must-Have Learning Settings And
    Default Parameter Values:
        ddpg_learning_setting = {}
        ddpg_learning_setting['env'] = createEnv()
        ddpg_learning_setting['env_eval'] = createEnv(eval=True)
        ddpg_learning_setting['learning_rate'] = 1e-2
        ddpg_learning_setting['discount_rate'] = 0.99
        ddpg_learning_setting['nn_layer_size'] = 8
        ddpg_learning_setting['n_steps'] = 100
        ddpg_learning_setting['tb_log_name'] = "ddpg-quad"
        ddpg_learning_setting['tb_log_dir'] = "./logs/quad_tensorboard/"
        ddpg_learning_setting['total_timesteps'] = total_timesteps
        ddpg_learning_setting['seed'] = None
        ddpg_learning_setting['policy'] = "MlpPolicy"
        ddpg_learning_setting['save'] = False
        ddpg_learning_setting['device'] = 'cpu'
    """
    env = learning_setting['env']

    policy_kwargs = dict(net_arch=[learning_setting['nn_layer_size'],
                                   learning_setting['nn_layer_size']])

    model = DDPG(learning_setting['policy'], env, verbose=0,
                 tensorboard_log=learning_setting['tb_log_dir'],
                 seed=learning_setting['seed'],
                 policy_kwargs=policy_kwargs,
                 learning_rate=learning_setting['learning_rate'],
                 gamma=learning_setting['discount_rate'],
                 batch_size=learning_setting['batch_size'],
                 device=learning_setting['device'])

    model.learn(total_timesteps=learning_setting['total_timesteps'],
                tb_log_name=learning_setting['tb_log_name'],
                callback=TensorboardCallback(learning_setting['tb_log_name']),
                eval_env=learning_setting['env_eval'],
                eval_freq=5000,
                n_eval_episodes=1)

    if learning_setting['save']:
        model.save("saves/" + learning_setting['tb_log_name'] +
                   "/" + "_end_" + str(datetime.datetime.now())+".zip")


def train_ppo_agent(learning_setting):
    """
    PPO(Proximal Policy Optimization Algorithm, Must-Have Learning Settings And
    Default Parameter Values:
        ppo_learning_setting = {}
        ppo_learning_setting['env'] = createEnv()
        ppo_learning_setting['env_eval'] = createEnv(eval=True)
        ppo_learning_setting['learning_rate'] = 1e-2
        ppo_learning_setting['discount_rate'] = 0.99
        ppo_learning_setting['nn_layer_size'] = 8
        ppo_learning_setting['n_steps'] = 2048
        ppo_learning_setting['tb_log_name'] = "ppo-quad"
        ppo_learning_setting['tb_log_dir'] = "./logs/quad_tensorboard/"
        ppo_learning_setting['total_timesteps'] = total_timesteps
        ppo_learning_setting['seed'] = None
        ppo_learning_setting['policy'] = "MlpPolicy"
        ppo_learning_setting['save'] = False
        ppo_learning_setting['device'] = 'cpu'
    """
    env = learning_setting['env']

    policy_kwargs = dict(net_arch=[
                         dict(pi=[learning_setting['nn_layer_size'],
                                  learning_setting['nn_layer_size']],
                              vf=[learning_setting['nn_layer_size'],
                                  learning_setting['nn_layer_size']])])

    model = PPO(learning_setting['policy'], env, verbose=0,
                tensorboard_log=learning_setting['tb_log_dir'],
                seed=learning_setting['seed'],
                policy_kwargs=policy_kwargs,
                learning_rate=learning_setting['learning_rate'],
                gamma=learning_setting['discount_rate'],
                n_steps=learning_setting['n_steps'],
                device=learning_setting['device'])

    model.learn(total_timesteps=learning_setting['total_timesteps'],
                tb_log_name=learning_setting['tb_log_name'],
                callback=TensorboardCallback(learning_setting['tb_log_name']),
                eval_env=learning_setting['env_eval'],
                eval_freq=5000,
                n_eval_episodes=1)

    if learning_setting['save']:
        model.save("saves/" + learning_setting['tb_log_name'] +
                   "/" + "_end_" + str(datetime.datetime.now())+".zip")


def train_sac_agent(learning_setting):
    """
    SAC Must-Have Learning Settings
    Default Parameter Values:
        sac_learning_setting = {}
        sac_learning_setting['env'] = createEnv()
        sac_learning_setting['env_eval'] = createEnv(eval=True)
        sac_learning_setting['learning_rate'] = 1e-2
        sac_learning_setting['discount_rate'] = 0.99
        sac_learning_setting['nn_layer_size'] = 8
        sac_learning_setting['n_steps'] = 100
        sac_learning_setting['tb_log_name'] = "sac-quad"
        sac_learning_setting['tb_log_dir'] = "./logs/quad_tensorboard/"
        sac_learning_setting['total_timesteps'] = total_timesteps
        sac_learning_setting['seed'] = None
        sac_learning_setting['policy'] = "MlpPolicy"
        sac_learning_setting['save'] = False
        sac_learning_setting['device'] = 'cpu'
    """

    env = learning_setting['env']

    policy_kwargs = dict(net_arch=[learning_setting['nn_layer_size'],
                                   learning_setting['nn_layer_size']])

    model = SAC(learning_setting['policy'], env, verbose=0,
                tensorboard_log=learning_setting['tb_log_dir'],
                seed=learning_setting['seed'],
                policy_kwargs=policy_kwargs,
                learning_rate=learning_setting['learning_rate'],
                gamma=learning_setting['discount_rate'],
                batch_size=learning_setting['batch_size'],
                device=learning_setting['device'])

    model.learn(total_timesteps=learning_setting['total_timesteps'],
                tb_log_name=learning_setting['tb_log_name'],
                callback=TensorboardCallback(learning_setting['tb_log_name']),
                eval_env=learning_setting['env_eval'],
                eval_freq=5000,
                n_eval_episodes=1)

    if learning_setting['save']:
        model.save("saves/" + learning_setting['tb_log_name'] +
                   "/" + "_end_" + str(datetime.datetime.now())+".zip")

    return model


def train_td3_agent(learning_setting):
    """
    TD3 Must-Have Learning Settings
        td3_learning_setting = {}
        td3_learning_setting['env'] = createEnv()
        td3_learning_setting['env_eval'] = createEnv(eval=True)
        td3_learning_setting['learning_rate'] = 1e-2
        td3_learning_setting['discount_rate'] = 0.99
        td3_learning_setting['nn_layer_size'] = 8
        td3_learning_setting['n_steps'] = 100
        td3_learning_setting['tb_log_name'] = "td3-quad"
        td3_learning_setting['tb_log_dir'] = "./logs/quad_tensorboard/"
        td3_learning_setting['total_timesteps'] = total_timesteps
        td3_learning_setting['seed'] = None
        td3_learning_setting['policy'] = "MlpPolicy"
        td3_learning_setting['save'] = False
        td3_learning_setting['device'] = 'cpu'
    """

    env = learning_setting['env']

    policy_kwargs = dict(net_arch=[learning_setting['nn_layer_size'],
                                   learning_setting['nn_layer_size']])

    model = TD3(learning_setting['policy'], env, verbose=0,
                tensorboard_log=learning_setting['tb_log_dir'],
                seed=learning_setting['seed'],
                policy_kwargs=policy_kwargs,
                learning_rate=learning_setting['learning_rate'],
                gamma=learning_setting['discount_rate'],
                batch_size=learning_setting['batch_size'],
                device=learning_setting['device'])

    model.learn(total_timesteps=learning_setting['total_timesteps'],
                tb_log_name=learning_setting['tb_log_name'],
                callback=TensorboardCallback(learning_setting['tb_log_name']),
                eval_env=learning_setting['env_eval'],
                eval_freq=5000,
                n_eval_episodes=1)

    if learning_setting['save']:
        model.save("saves/" + learning_setting['tb_log_name'] +
                   "/" + "_end_" + str(datetime.datetime.now())+".zip")

    return model


def train_a2c_agent(learning_setting):
    """
    A2C(Advantage Actor Critic Algorithm, Must-Have Learning Settings And
    Default Parameter Values:
        a2c_learning_setting = {}
        a2c_learning_setting['env'] = createEnv()
        a2c_learning_setting['env_eval'] = createEnv(eval=True)
        a2c_learning_setting['learning_rate'] = 1e-2
        a2c_learning_setting['discount_rate'] = 0.99
        a2c_learning_setting['nn_layer_size'] = 8
        a2c_learning_setting['n_steps'] = 5
        a2c_learning_setting['tb_log_name'] = "a2c-quad"
        a2c_learning_setting['tb_log_dir'] = "./logs/quad_tensorboard/"
        a2c_learning_setting['total_timesteps'] = total_timesteps
        a2c_learning_setting['seed'] = None
        a2c_learning_setting['policy'] = "MlpPolicy"
        a2c_learning_setting['save'] = False
        a2c_learning_setting['device'] = 'cpu'
    """

    env = learning_setting['env']

    policy_kwargs = dict(net_arch=[learning_setting['nn_layer_size'],
                                   learning_setting['nn_layer_size']])

    model = A2C(learning_setting['policy'], env, verbose=0,
                tensorboard_log=learning_setting['tb_log_dir'],
                seed=learning_setting['seed'],
                policy_kwargs=policy_kwargs,
                learning_rate=learning_setting['learning_rate'],
                gamma=learning_setting['discount_rate'],
                n_steps=learning_setting['n_steps'],
                device=learning_setting['device'])

    model.learn(total_timesteps=learning_setting['total_timesteps'],
                tb_log_name=learning_setting['tb_log_name'],
                callback=TensorboardCallback(learning_setting['tb_log_name']),
                eval_env=learning_setting['env_eval'],
                eval_freq=5000,
                n_eval_episodes=1)

    if learning_setting['save']:
        model.save("saves/" + learning_setting['tb_log_name'] +
                   "/" + "_end_" + str(datetime.datetime.now())+".zip")

    return model


class TensorboardCallback(BaseCallback):
    """
    Custom callback for plotting additional values in tensorboard.
    """

    def __init__(self, name_prefix, verbose=0, auto_save=True,
                 save_path="saves/", save_freq=50_000):
        super(TensorboardCallback, self).__init__(verbose)
        self.name_prefix = name_prefix
        self.auto_save = auto_save
        self.save_path = save_path
        self.save_freq = save_freq

    def _on_training_start(self):
        output_formats = self.logger.output_formats
        # Save reference to tensorboard formatter object
        # note: the failure case (not formatter found) is
        # not handled here, should be done with try/except.
        self.tb_formatter = next(formatter for formatter in output_formats
                                 if isinstance(formatter,
                                               TensorBoardOutputFormat))

    def _on_step(self) -> bool:
        if self.auto_save:
            if self.n_calls % self.save_freq == 0:
                path = os.\
                    path.\
                    join(self.save_path,
                         f"{self.name_prefix}/{self.name_prefix}_" +
                         f"{self.num_timesteps}_steps")
                self.model.save(path)
                if self.verbose > 1:
                    print(f"Saving model checkpoint to {path}")
        return True
