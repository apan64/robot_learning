import numpy as np
import wave

class LinearRegressionLearning():
    def __init__(self):
        pass

    def calculate_offset(self, data_0, data_1):
        '''
        Calculate the offset of the data by using cross-correlation
        Returns a number indicating the offset of the two input data, positive means data_0 needs to be shifted left to match data_1, negative means shift right
        '''
        correlated = np.correlate(data_0, data_1, mode='full')
        return np.argmax(correlated) - (len(data_1) - 1)

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


if __name__ == '__main__':
    pass