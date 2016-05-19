# coding: utf-8

# Lindsay Kleeman, "Understanding and Applying Kalman Filtering"
# , "Department of Electrical and Computer Systems Engineering
# Monash University, Clayton"
# , "http://biorobotics.ri.cmu.edu/papers/sbp_papers/integrated3
# /kleeman_kalman_basics.pdf"

class LinearKalmanFilter:
	# init
	def __init__(self, x, P, F, G, u, H, Q, R):
		self.x = x
		self.P = P
		
		self.F = F
		self.G = G
		self.u = u
		self.H = H
		self.Q = Q
		self.R = R
	
	# update
	def update(self, z):
		# State Prediction
		x_ = self.F * self.x + self.G * self.u
		
		# Measurement Prediction
		z_ = self.H * x_
		
		# Measurement Residual
		v = z - z_
		
		# State predication covariance
		P_ = self.F * self.P * self.F.T + self.Q
		
		# Measurement predication covariance
		S = self.H * P_ * self.H.T + self.R
		
		# Filter Gain(Kalman Gain)
		K = P_ * self.H.T * S.I
		
		# Updated state covariance
		self.P = P_ - K * self.H * P_
		#self.P = P_ - K * S * K.T (mistake ?)
		
		# Updated State Estimate
		self.x = x_ + K * v
		
		return self.x