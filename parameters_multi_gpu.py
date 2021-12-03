from parameters import ddpg_learning_setting
from parameters import ppo_learning_setting
from parameters import sac_learning_setting
from parameters import a2c_learning_setting
from parameters import td3_learning_setting

ddpg_learning_setting['device'] = 'cuda:0'
ppo_learning_setting['device'] = 'cuda:1'
sac_learning_setting['device'] = 'cuda:2'
td3_learning_setting['device'] = 'cuda:3'
a2c_learning_setting['device'] = 'cuda:0'
