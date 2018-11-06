import numpy as np
from scipy import fft, ifft, conj
from scipy.io.wavfile import read
from sklearn.preprocessing import normalize

class LinearRegressionLearning():
    def __init__(self, num_weights=2):
        self.weights = np.ones(num_weights)
        self.stored_data = None

    def extract_channel_data(self, filename):
        '''
        Operating under assumption of all files having been recorded from the microphone array connected to raspi
        Returns tuple containing two numpy ndarrays, each corresponding to a channel (left or right microphone data)
        ndarrays contain one datapoint per frame, our files have 44100 fps, recordings 3s
        '''
        data = read(filename)[1]
        return data[:, 0], data[:, 1]

    def calculate_offset(self, data_0, data_1):
        '''
        Calculate the offset of the data by using FFT and convolution
        Returns the offset in frames
        reference for FFT stuffs https://stackoverflow.com/a/4688875/10582078
        # OLD # reference for determining shift instead of the severely lacking numpy.correlate documentation: https://stackoverflow.com/questions/49372282/find-the-best-lag-from-the-numpy-correlate-output?newreg=0cb46c75c1e842649a5c3996e2ce79b5
        '''
        convolved = ifft(fft(data_0) * conj(fft(data_1)))
        abs_convolved = np.absolute(convolved)
        offset = np.argmax(abs_convolved)
        if len(data_0) - offset < offset:
            return offset - len(data_0)
        return offset


    def prepare_data(self, data_0, data_1, expected):
        '''
        Given channel data for audio, returns the offset between the two channels, the average value of all the data in the channels, and the expected angle of the snap relative to the robot
        '''
        offset = self.calculate_offset(data_0, data_1)
        return offset, (np.average(data_0) + np.average(data_1)) / 2, expected

    def store_data(self, data):
        '''
        Store data in class property, along with the expected output value of the data
        Data of the format np.ndarray, each row is np.array([correlation offset, average wav value of the file, expected output])
        '''
        if type(self.stored_data) == np.ndarray:
            self.stored_data = np.vstack([self.stored_data, data])
        else: # Catches weight initialized as None and initializes it as numpy.ndarray
            self.stored_data = np.ndarray(shape=(1, 3), dtype=np.float32)
            self.stored_data[0] = np.array(data)

    def normalize(self):
        '''
        Returns a normalized version of the data stored in the class
        '''
        # return np.hstack((normalize(np.delete(self.stored_data, 2, axis=1), axis=0, norm='l1'), self.stored_data[:, [2]]))
        return normalize(self.stored_data, axis=0, norm='l1')

    def calculate_loss(self):
        '''
        |   | i

        ||  | __

        Runs through current stored data using weights to calculate values for each set of stored data, then calculates the loss for each data
        Returns average of the losses
        '''
        normalized_data = self.normalize()
        features = np.delete(normalized_data, 2, axis=1)
        targets = normalized_data[:,2]
        predictions = self.predict(features, self.weights)
        N = len(features)

        # Matrix math
        sq_error = (predictions - targets)**2
        # Return average squared error among predictions
        return 1.0/(2*N) * sq_error.sum()
        

    def predict(self, features, weights):
        '''
        Performs matrix multiplication between the features and weights to obtain predictions
        '''
        return np.dot(features,weights)



    def adjust_weights(self):
        '''
        Adjusts weights using gradient descent
        All data should be stored in the class before running
        '''

        normalized_data = self.normalize()
        delays = normalized_data[:, 0]
        wavs = normalized_data[:, 1]

        features = np.delete(normalized_data, 2, axis=1)
        targets = normalized_data[:, 2]
        predictions = self.predict(features, self.weights)

        d_delay = -delays * (targets - predictions)
        d_wav = -wavs * (targets - predictions)

        print('Features: {}\nWeights: {}\nTargets: {}\nPredictions: {}'.format(features, self.weights, targets, predictions))
        print("d_delay:{}, d_wav{}".format(d_delay, d_wav))
        self.weights[0] -= np.mean(d_delay)
        self.weights[1] -= np.mean(d_wav)





if __name__ == '__main__':
    lin = LinearRegressionLearning()
    files = open('data/recordings2/training_filename_list.txt', 'r') # Change this to be the txt file with names of wav files
    audio_string = 'audio'
    for file in files:
        wav_filename = file.strip()
        channel_0, channel_1 = lin.extract_channel_data('data/recordings2/{}'.format(wav_filename))
        expected_val = float(wav_filename[wav_filename.index(audio_string) + len(audio_string) + 2 :len(wav_filename)-4])
        lin.store_data(lin.prepare_data(channel_0, channel_1, expected_val))
        lin.adjust_weights()
        print('Angle: {}\tLoss: {}\n\n'.format(expected_val, lin.calculate_loss()))