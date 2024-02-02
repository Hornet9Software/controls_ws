'''
The Kalman Filter relates the state at the previous timestep to the current timestep. We can predict current state using previous measurements, expected equations of motions or input and current measurements.

Prediction step: Using the previous mean and covariance of state X,
predict mean and covariance of current state before seeing measurements.

Update step: Estimate current state using measurements at current timestep and the predicted mean and covariance in prediction step.

Code is divided into three functions.

'''
from numpy import dot, sum, tile, linalg, log, pi, exp, zeros, eye
from numpy.linalg import inv, det

import numpy as np

class KalmanFilter:
    def __init__(self, name) -> None:
       self.name = name


    '''
    X: mean of state at timestep k-1
    P: covariance of state at timestamp k-1
    A: transition n x n matrix
    Q: process noise covariance matrix
    B: input effect matrix
    U: control input (from us!)
    '''
    def kf_predict(self, X, P, A, Q, B, U):
        X = dot(A, X) + dot(B, U)
        P = dot(A, dot(P, A.T)) + Q
        return (X, P) # state mean and covariance at timestep k

    '''
    X and P are from the prediction step
    K: Kalman Gain matrix
    Y: measurement vector
    H: measurement matrix
    R: measurement covariance matrix
    IM: mean of measurement Y (distribution)
    IS: covariance/predictive mean of Y
    LH: predictive probability (likelihood) of measurement computed
        with gauss_pdf
    '''
    def kf_update(self, X, P, Y, H, R):
        IM = dot(H, X)
        IS = R + dot(H, dot(P, H.T))
        K = dot(P, dot(H.T, inv(IS)))
        X = X + dot(K, (Y - IM))
        P = P - dot(K, dot(IS, K.T))
        LH = self.gauss_pdf(Y, IM, IS)
        return (X, P, K, IM, IS, LH)
    

    def gauss_pdf(self, X, M, S):
        if M.shape()[1] == 1:
            DX = X - tile(M, X.shape()[1])
            E = 0.5 * sum(DX * (dot(inv(S), DX)), axis=0)
            E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(det(S))
            P = exp(-E)
        elif X.shape()[1] == 1:
            DX = tile(X, M.shape()[1]) - M
            E = 0.5 * sum(DX * (dot(inv(S), DX)), axis=0)
            E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(det(S))
            P = exp(-E)
        else:
            DX = X - M
            E = 0.5 * dot(DX.T, dot(inv(S), DX))
            E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(det(S))
            P = exp(-E)
        
        return (P[0], E[0])
    

class KalmanFilter2:
    '''
    dim_x corresponds to the number of state variables. in our case 18
    because of linear/angular position/velocity/acceleration

    dim_z corresponds to incoming measurement. looking at IMU, we
    should have 9
    '''
    def __init__(self, dim_x = 18, dim_z = 9):
        
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.x = zeros((dim_x, 1)) # state vector/mean
        self.P = eye(dim_x) # identity matrix. state covariance matrix
        self.Q = eye(dim_x) # noise covariance matrix
        self.F = eye(dim_x) # process model transistion matrix
        self.H = zeros((dim_z, dim_x)) # measurement transition matrix
        self.R = eye(dim_z) # sensor noise covariance
        self.M = zeros((dim_z, dim_z))

        self._I = eye(dim_x) # Helps I matrix to be compatible with state vector dims

        self.x_prior = np.copy(self.x)
        self.P_prior = np.copy(self.P)

    def predict(self):
        '''
        Predict next state (prior)
        '''

        self.x = dot(self.F, self.X)
        self.P = dot(self.F, dot(self.P, self.F.T)) + self.Q
        self.x_prior = np.copy(self.x)
        self.P_prior = np.copy(self.P)

    def update(self, z):
        '''
        At time step k, update computes posterior mean x and covariance
        P using measurement z
        '''

        # residual between measurement and prediction
        y = z - dot(self.H, self.x)
        PHT = dot(self.P, self.H.T)

        # Project system uncertainty into measurement space
        S = dot(self.H, PHT) + self.R

        # map system uncertainty into Kalman gain
        K = dot(PHT, inv(S))

        # predict new x with residual scaled by Kalman gain
        self.x = self.x + dot(K, y)

        I_KH = self._I - dot(K, self.H)
        self.P = dot(I_KH, self.P)