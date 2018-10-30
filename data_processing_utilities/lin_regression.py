import numpy as np
import wave

class LinearRegressionLearning():
    def __init__(self, num_weights = 10):
        self.weights = np.array([0 for __ in range(num_weights)])
        self.stored_data = []

    def extract_channel_data(self, filename):
        '''
        Operating under assumption of all files having been recorded from the microphone array connected to raspi
        Returns tuple containing two numpy ndarrays, each corresponding to a channel (left or right microphone data)
        ndarrays contain one datapoint per frame, our files have 44100 fps, recordings 3s
        '''
        w_read = wave.open(filename, 'r')
        data = np.fromstring(w_read.readframes(w_read.getnframes()), dtype=np.uint32)
        channel_data_0 = data[0::2]
        channel_data_1 = data[1::2]
        return channel_data_0, channel_data_1

    def calculate_offset(self, data_0, data_1):
        '''
        Calculate the offset of the data by using cross-correlation
        Returns a number indicating the offset of the two input data, positive means data_0 needs to be shifted left to match data_1, negative means shift right
        reference for determining shift instead of the severely lacking numpy.correlate documentation: https://stackoverflow.com/questions/49372282/find-the-best-lag-from-the-numpy-correlate-output?newreg=0cb46c75c1e842649a5c3996e2ce79b5
        '''
        correlated = np.correlate(data_0, data_1, mode='full')
        return np.argmax(correlated) - (len(data_1) - 1)

    def store_data(self, data):
        '''
        Store data in class property, along with the expected output value of the data
        '''
        pass

    def calculate_loss(self):
        '''
        |   | i

        ||  | __

        Runs through current stored data using weights to calculate values for each set of stored data, then calculates the loss for each data
        Returns average of the losses
        '''
        pass

    def adjust_weights(self):
        '''
        Adjust weights based on loss value
        Use the averages of the stored data at each index
        '''
        pass


if __name__ == '__main__':
    pass