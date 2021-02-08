import numpy as np
import matplotlib.pyplot as plt


class MotionModel():                                        #advance the state model
    def __init__(self, A, Q):
        self.A = A                                          #initialize motion matrix
        self.Q = Q                                          #initialize gaussian matrix

        (m, _) = Q.shape                                    #grab the number of rows and put it in m
        self.zero_mean = np.zeros(m)                        #make a zero matrix of the shape of number of rows

    def __call__(self, x):
        new_state = self.A @ x + np.random.multivariate_normal(self.zero_mean, self.Q)      #calculate the new state from the motion on the current state plus randomness
        return new_state


class MeasurementModel():                               #make measurements
    def __init__(self, H, R):
        self.H = H                                      #initialize measurement matrix
        self.R = R                                      #initialize measurement uncertainty

        (n, _) = R.shape                                #get number of rows
        self.zero_mean = np.zeros(n)                    #make matrix of that size

    def __call__(self, x):
        measurement = self.H @ x + np.random.multivariate_normal(self.zero_mean, self.R)    #calculate measurement data by measuring the state and adding noise
        return measurement


def create_model_parameters(T, sigma2, lambda2):                   #model parameters and defaults
    # Motion model parameters
    F = np.array([[1, T],                                                                           #timestep matrix
                  [0, 1]])
    base_sigma = np.array([[T ** 3 / 3, T ** 2 / 2],                                                #discretized gaussian
                           [T ** 2 / 2, T]])

    sigma_x = sigma2 * base_sigma                                                                     #stretch each dimension according to noise

    A = F
    Q = sigma_x                                                              #generate 2d sigma matrix

    # Measurement model parameters
    H = np.eye(2)
    R = lambda2 * np.eye(2)                                                                         #create 2D noise matrix

    return A, H, Q, R


def simulate_system(K, x0):                                                     #run this, input kalman gain and the first state
    (A, H, Q, R) = create_model_parameters()                                    #create evolution, measurement, model noise, and measurement noise

    # Create models
    motion_model = MotionModel(A, Q)                                            #fill model and model noise
    meas_model = MeasurementModel(H, R)                                         #fill measurement and measurement noise

    (m, _) = Q.shape
    (n, _) = R.shape

    state = np.zeros((K, m))
    meas = np.zeros((K, n))

    # initial state
    x = x0
    for k in range(K):                                                          #do this for each element in the kalman weight matrix
        x = motion_model(x)                                                     #go to the next state, update the model with noise
        z = meas_model(x)                                                       #go to the next state update the measurement with noise

        state[k, :] = x                                                         #write in the new state
        meas[k, :] = z                                                          #record the new measurement

    return state, meas


if __name__ == '__main__':
    np.random.seed(21)
    (state, meas) = simulate_system(K=20, x0=np.array([0, 0.1, 0, 0.1]))        #i dont think K is the kalman matrix here

    plt.figure(figsize=(7, 5))
    plt.plot(state[:, 0], state[:, 2], '-bo')
    plt.plot(meas[:, 0], meas[:, 1], 'rx')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.legend(['true state', 'observed measurement'])
    plt.axis('square')
    plt.tight_layout(pad=0)
    plt.show()

