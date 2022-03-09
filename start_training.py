import sys
import torch.multiprocessing as mp
from src.UtilStableAgents import train_ddpg_agent, train_ppo_agent
from src.UtilStableAgents import train_sac_agent, train_td3_agent
from src.UtilStableAgents import train_a2c_agent

if __name__ == '__main__':
    from parameters import number_of_parallel_experiments
    from parameters import start_ddpg, start_ppo
    from parameters import start_sac, start_td3
    from parameters import start_a2c

    params = len(sys.argv)
    if params == 2 and sys.argv[1] == 'multigpu':
        from parameters_multi_gpu import ddpg_learning_setting
        from parameters_multi_gpu import ppo_learning_setting
        from parameters_multi_gpu import sac_learning_setting
        from parameters_multi_gpu import a2c_learning_setting
        from parameters_multi_gpu import td3_learning_setting
    else:
        from parameters import ddpg_learning_setting
        from parameters import ppo_learning_setting
        from parameters import sac_learning_setting
        from parameters import a2c_learning_setting
        from parameters import td3_learning_setting

    mp.set_start_method('spawn')
    processes = []

    for rank in range(number_of_parallel_experiments):

        if start_ddpg:
            p = mp.Process(target=train_ddpg_agent,
                           kwargs={'learning_setting': ddpg_learning_setting})
            p.start()
            processes.append(p)

        if start_ppo:
            p = mp.Process(target=train_ppo_agent,
                           kwargs={'learning_setting': ppo_learning_setting})
            p.start()
            processes.append(p)

        if start_sac:
            p = mp.Process(target=train_sac_agent,
                           kwargs={'learning_setting': sac_learning_setting})
            p.start()
            processes.append(p)

        if start_a2c:
            p = mp.Process(target=train_a2c_agent,
                           kwargs={'learning_setting': a2c_learning_setting})
            p.start()
            processes.append(p)

        if start_td3:
            p = mp.Process(target=train_td3_agent,
                           kwargs={'learning_setting': td3_learning_setting})
            p.start()
            processes.append(p)

    for p in processes:
        p.join()
