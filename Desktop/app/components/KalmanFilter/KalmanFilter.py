# kalman1d.py

class KalmanFilter1D:
    def __init__(self, initial_state=0.0, initial_uncertainty=1.0, process_noise=1.0, measurement_noise=9.0):

        self.state = initial_state
        self.uncertainty = initial_uncertainty
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise

    def update(self, control_input, measurement):

        # Predição
        self.state += 0.004 * control_input
        
        # Atualização da incerteza com o ruído de processo
        self.uncertainty += self.process_noise

        # Cálculo do ganho de Kalman
        kalman_gain = self.uncertainty / (self.uncertainty + self.measurement_noise)

        # Correção do estado com a medição
        self.state += kalman_gain * (measurement - self.state)

        # Atualização da incerteza
        self.uncertainty = (1 - kalman_gain) * self.uncertainty

        return self.state, self.uncertainty
