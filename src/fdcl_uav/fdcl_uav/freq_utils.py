

class Freq():
    def __init__(self, name, freq):
        self.name = name
        self.t_prev = 0.0
        self.num_avg = 50
        self.freq_avg = freq
        self.dt = 0

    def update(self, t_now):
        dt = t_now - self.t_prev

        if dt <= 0.0:
            return
        
        self.dt = dt
        freq = 1.0 / dt
        self.freq_avg = (freq + (self.num_avg - 1) * self.freq_avg) / self.num_avg

        self.t_prev = t_now


    def get_freq(self):
        return self.freq_avg
    

    def get_freq_str(self):
        return f'{self.name}: {self.freq_avg:5.1f} Hz'