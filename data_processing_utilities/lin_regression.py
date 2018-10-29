import numpy as np
import wave

class LinearRegressionLearning():
    def __init__(self):
        pass

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