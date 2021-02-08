import gym
import numpy as np 
import matplotlib.pyplot as plt 

MAXSTATES = 1000                                    #1k step in our parameter space


def max_dict(Q,state):
	max_v = float('-inf')
	for i in range(2):								#only two possible actions
		if Q[int(state)][i] > max_v:
			max_v = Q[int(state)][i]
			max_index = i
	return max_v , max_index

def create_bins():									#use binning to go from continuous space to discrete
	# obs[0] -> cart position --- -4.8 - 4.8
	# obs[1] -> cart velocity --- -inf - inf
	# obs[2] -> pole angle    --- -41.8 - 41.8
	# obs[3] -> pole velocity --- -inf - inf
	
	bins = np.zeros((3,10))							#create a bin for each parameter in that space (block position, block velocity, stick angle and stick velocity)
	bins[0] = np.linspace(-1, 1, 10)
	bins[1] = np.linspace(-1, 1, 10)
	bins[2] = np.linspace(-1, 1, 10)

	return bins

def assign_bins(observation, bins):					#quickly place the observed value into a bin
	state = np.zeros(3)
	for i in range(3):
		state[i] = np.digitize(observation[i], bins[i])
	return state

def get_state_as_string(state):						#makes sure that the states come back in a uniform string
	string_state = ''.join(str(int(e)) for e in state)
	return string_state



def initialize_Q():									#build the state space. Basically go through every possible bin(x2 for each action) and set it to 
	Q = np.zeros((MAXSTATES,2))						#initialize all scores
	return Q

