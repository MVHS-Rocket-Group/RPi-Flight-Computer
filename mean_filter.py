class IMUFilter:
    def __init__(self, filter_buffer_size: int):
        self.acc_history = []
        self.gyro_history = []
        self.mag_history = []
        self.filter_buffer_size = filter_buffer_size

    def add_data(self, acc, gyro, mag):
        self.acc_history.append(acc)
        self.gyro_history.append(gyro)
        self.mag_history.append(mag)

        if len(self.acc_history) > self.filter_buffer_size:
            del self.acc_history[0]

        if len(self.gyro_history) > self.filter_buffer_size:
            del self.gyro_history[0]

        if len(self.mag_history) > self.filter_buffer_size:
            del self.mag_history[0]

    def update_filter(self):
        acc_total = [0.0, 0.0, 0.0]
        for element in self.acc_history:
            acc_total[0] += element[0]
            acc_total[1] += element[1]
            acc_total[2] += element[2]

        gyro_total = [0.0, 0.0, 0.0]
        for element in self.gyro_history:
            gyro_total[0] += element[0]
            gyro_total[1] += element[1]
            gyro_total[2] += element[2]

        mag_total = [0.0, 0.0, 0.0]
        for element in self.mag_history:
            mag_total[0] += element[0]
            mag_total[1] += element[1]
            mag_total[2] += element[2]

        acc_len = len(self.acc_history)
        acc_avg = [acc_total[0] / acc_len, acc_total[1] /
                   acc_len, acc_total[2] / acc_len]

        gyro_len = len(self.gyro_history)
        gyro_avg = [gyro_total[0] / gyro_len, gyro_total[1] /
                    gyro_len, gyro_total[2] / gyro_len]

        mag_len = len(self.mag_history)
        mag_avg = [mag_total[0] / mag_len, mag_total[1] /
                   mag_len, mag_total[2] / mag_len]

        return (acc_avg, gyro_avg, mag_avg)
