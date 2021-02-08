import numpy as np


class KalmanFilter():
    def __init__(self, A, H, Q, R, x_0, P_0):                                   #initialize kalman filter
        # Model parameters
        self.A = A
        self.H = H
        self.Q = Q
        self.R = R

        # Initial state
        self._x = x_0                                                   #fancy state        
        self._P = P_0                                                       #gaussian error matrix

    def predict(self):
        self._x = self.A @ self._x                                              #evolve state
        self._P = self.A @ self._P @ self.A.transpose() + self.Q                #evolve error in model

    def update(self, z,l2):
        self.R = l2 * np.eye(2)                                                     #update 0th and 1st derivative std. dev.
        self.S = self.H @ self._P @ self.H.transpose() + self.R                     #do a measurement of the model gaussian and add measurement noise
        self.V = z - self.H @ self._x                                               #find the innovation (compare real measurement to predicted measurement)
        self.K = self._P @ self.H.transpose() @ np.linalg.inv(self.S)               #actual kalman weights matrix for the predictions and the model gaussian

        self._x = self._x + self.K @ self.V                                         #update the predicted model using the weights and the observiations
        self._P = self._P - self.K @ self.S @ self.K.transpose()                    #update the prediction gaussian using the weights and the gaussian noise

    def update_time(self,T,sigma2):
        self.A = np.array([[1, T],                                                                           #timestep matrix
                           [0, 1]])
        base_sigma = np.array([[T ** 3 / 3, T ** 2 / 2],                                                #discretized gaussian
                               [T ** 2 / 2, T]])
        self.Q = sigma2 * base_sigma                                                                     #stretch each dimension according to noise


    def get_state(self):
        return self._x, self._P                                                     #return state and gaussian
