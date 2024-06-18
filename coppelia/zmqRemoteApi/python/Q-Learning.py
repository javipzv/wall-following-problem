'''
Q-Learning.py

Implementation of Q-Learning algorithm to solve the problem 
of the robot avoiding obstacles.

Copyright (C) 2024 Javier Pérez Vargas

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

import robotica
import numpy as np
import matplotlib.pyplot as plt
import time

def initialize_position(robot):
    initial_positions = [(-1.74447, +1.750), (+4.78053, -1.875), (+4.68053, +0.550)]
    random_pos = initial_positions[np.random.randint(0, len(initial_positions))]
    robot.set_position(random_pos[0], random_pos[1])

def learn(qtable, state_discrete, action_index, reward, next_state_discrete, alpha, gamma):
    if state_discrete not in qtable:
        qtable[state_discrete] = np.zeros(N_ACTIONS)
    if next_state_discrete not in qtable:
        qtable[next_state_discrete] = np.zeros(N_ACTIONS)
    qtable[state_discrete][action_index] = (1 - alpha) * qtable[state_discrete][action_index] + alpha * (reward + gamma * np.max(qtable[next_state_discrete]))

def select_action(qtable, state_discrete, exploration_rate):
    if np.random.rand() < exploration_rate:
        return np.random.randint(0, len(ACTIONS))
    else:
        if state_discrete not in qtable:
            qtable[state_discrete] = np.zeros(N_ACTIONS)
        return np.argmax(qtable[state_discrete])

def discretize_state(state):
    discrete_state = ""
    for i in range(8):
        if state[i] < 0.3:
            discrete_state += "0"
        elif state[i] < 0.6:
            discrete_state += "1"
        else:
            discrete_state += "2"
    return discrete_state

def main(args=None):
    """Constants"""
    global ACTIONS, N_STATES, N_ACTIONS, EPSILON, EPSILON_DECAY, EPSILON_MIN, ALPHA, GAMMA
    EPISODES = 200
    ACTIONS = [(-1, 1), (0, 1), (2, 2), (1.5, 1.5), (2, 2), (2.5, 2.5), (1, 0), (1, -1)]
    N_STATES = 3**8
    N_ACTIONS = len(ACTIONS)
    EPSILON = 1
    EPSILON_DECAY = 0.99
    EPSILON_MIN = 0.05
    ALPHA = 0.3
    GAMMA = 1

    """Initialization"""
    coppelia = robotica.Coppelia()
    robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX')
    qtable = {}
    array_acc_rewards = []

    """Training"""
    for i in range(EPISODES):
        coppelia.start_simulation()
        initialize_position(robot)
        state = robot.get_sonar()
        state_discrete = discretize_state(state)
        terminal = False
        acc_reward = 0
        while coppelia.is_running():
            
            # Epsilon greedy
            action_index = select_action(qtable, state_discrete, EPSILON)
            action = ACTIONS[action_index]

            # Execute action
            robot.set_speed(action[0], action[1])
            time.sleep(0.3)

            # Get new state and reward
            new_state = robot.get_sonar()
            new_state_discrete = discretize_state(new_state)

            if min(new_state[:8]) < 0.1:
                reward = -50
                terminal = True
            else:
                if new_state[2] < 0.3 or new_state[3] < 0.4 or new_state[4] < 0.4 or new_state[5] < 0.3:
                    reward = -5
                elif (new_state[5] > 0.4 and new_state[5] < 0.6) or (new_state[6] > 0.3 and new_state[6] < 0.6) or (new_state[7] > 0.3 and new_state[7] < 0.6):
                    reward = +20
                else:
                    reward = -1

            acc_reward += reward

            # Update Q-table
            learn(qtable, state_discrete, action_index, reward, new_state_discrete, ALPHA, GAMMA)

            # Update state
            state = new_state
            state_discrete = discretize_state(state)

            # Check if terminal state
            if terminal:
                break
            
        # Update exploration rate
        EPSILON *= EPSILON_DECAY
        EPSILON = max(EPSILON_MIN, EPSILON)

        coppelia.stop_simulation()

        print("Episode", i+1, "| Acc. Reward:", acc_reward, "| Epsilon:", round(EPSILON, 2))
        array_acc_rewards.append(acc_reward)

    plt.plot(array_acc_rewards)
    plt.show()
    accumulated_mean = []

    for i in range(1, len(array_acc_rewards)+1):
        accumulated_mean.append(sum(array_acc_rewards[max(0, i-20):i])/min(i, 20))

    plt.plot(accumulated_mean)
    plt.show()

if __name__ == '__main__':
    main()