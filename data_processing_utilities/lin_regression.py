import numpy as np
import wave
from scipy.io.wavfile import read

class LinearRegressionLearning():
    def __init__(self, num_weights=6):
        self.weights = np.array([1 for __ in range(num_weights)])
        self.stored_data = None

    def extract_channel_data(self, filename):
        '''
        Operating under assumption of all files having been recorded from the microphone array connected to raspi
        Returns tuple containing two numpy ndarrays, each corresponding to a channel (left or right microphone data)
        ndarrays contain one datapoint per frame, our files have 44100 fps, recordings 3s
        '''
        # w_read = wave.open(filename, 'r')
        # data = np.fromstring(w_read.readframes(w_read.getnframes()), dtype=np.uint32)
        # channel_data_0 = data[0::2]
        # channel_data_1 = data[1::2]
        channel_data_0, channel_data_1 = read(filename)[1]
        return channel_data_0, channel_data_1

    def calculate_offset(self, data_0, data_1):
        '''
        Calculate the offset of the data by using cross-correlation
        Returns a tuple consisting of a number indicating the offset of the two input data, positive means data_0 needs to be shifted left to match data_1, negative means shift right, and the actual correlation value
        reference for determining shift instead of the severely lacking numpy.correlate documentation: https://stackoverflow.com/questions/49372282/find-the-best-lag-from-the-numpy-correlate-output?newreg=0cb46c75c1e842649a5c3996e2ce79b5
        '''
        correlated = np.correlate(data_0, data_1, mode='full')
        return np.argmax(correlated) - (len(data_1) - 1), np.max(correlated)

    def prepare_data(self, data_0, data_1, expected):
        offset = self.calculate_offset(data_0, data_1)
        return offset[0], offset[1], (np.average(data_0) + np.average(data_1)) / 2, expected

    def store_data(self, data):
        '''
        Store data in class property, along with the expected output value of the data
        Data of the format np.ndarray, each row is np.array([correlation offset, correlation value, average wav value, expected output])
        '''
        # self.stored_data.append(data)
        if stored_data:
            self.stored_data = np.vstack([self.stored_data, data])
        else:
            self.stored_data = np.ndarray(shape=(1, 4), dtype=np.float32)
            self.stored_data[0] = np.array(data)

    def calculate_loss(self):
        '''
        |   | i

        ||  | __

        Runs through current stored data using weights to calculate values for each set of stored data, then calculates the loss for each data
        Returns average of the losses
        '''
        pass

    def adjust_weights(self, loss):
        '''
        Adjust weights based on loss value
        '''
        # # calculating the averages of all the data for each entry
        # delay_average, correlate_average = np.average(self.stored_data[:, 0]), np.average(self.stored_data[:, 1])
        # # adjusting weights based on average values and the total average loss


        # might just do actual gradient descent
        '''
        Features (data):(x, 3)
        Targets: (x, 1)
        Weights:(6, 1) - scratch that, (3, 1)
        '''
        delays = self.stored_data[:, 0]
        correlates = self.stored_data[:, 1]
        wavs = self.stored_data[:, 2]

        # d_delay_0 = -delays * (''' this needs to be the derivative of the loss function''')
        # # d_delay_1 = 
        # d_correlate_0 = 
        # # d_correlate_1 = 
        # d_wav_0 = 
        # # d_wav_1 = 





if __name__ == '__main__':
    pass