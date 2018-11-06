import numpy as np
from scipy import fft, ifft, conj
from scipy.io.wavfile import read
from sklearn.preprocessing import normalize
import matplotlib.pyplot as plt
import math

class LinearRegressionLearning():
    def __init__(self, num_weights=1):
        self.weights = np.ones(num_weights)
        self.weights = [1]
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
        # Returns a tuple consisting of a number indicating the offset of the two input data, positive means data_0 needs to be shifted left to match data_1, negative means shift right, and the actual correlation value
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
        offset = self.calculate_offset(data_0, data_1)
        # return offset[0], offset[1], (np.average(data_0) + np.average(data_1)) / 2, expected
        return offset, (np.average(data_0) + np.average(data_1)) / 2, expected

    def store_data(self, data):
        '''
        Store data in class property, along with the expected output value of the data
        Data of the format np.ndarray, each row is np.array([correlation offset, correlation value, average wav value, expected output])
        '''
        if type(self.stored_data) == np.ndarray:
            self.stored_data = np.vstack([self.stored_data, data])
        else: # Catches weight initialized as None and initializes it as numpy.ndarray
            self.stored_data = np.ndarray(shape=(1, 3), dtype=np.float32)
            self.stored_data[0] = np.array(data)

    def normalize(self):
        # features = np.delete(self.stored_data, 2, axis=1)
        # for feature in features.T:
        #     fmean = np.mean(feature)
        #     frange = np.amax(feature) - np.amin(feature)

        #     #Vector Subtraction
        #     feature -= fmean

        #     #Vector Division
        #     feature /= frange
        # print('THIS IS NORMALIZE \n___________________\nSTORED_DATA: {}\nFEATURES: {}'.format(self.stored_data, features))

        # self.stored_data = np.hstack((features, self.stored_data[:, [2]]))
        return np.hstack((normalize(np.delete(self.stored_data, 2, axis=1), axis=0, norm='l1'), self.stored_data[:, [2]]))

    def calculate_loss(self):
        '''
        |   | i

        ||  | __

        Runs through current stored data using weights to calculate values for each set of stored data, then calculates the loss for each data
        Returns average of the losses
        '''
        # normalized_data = self.normalize()
        normalized_data = self.stored_data
        features = np.delete(normalized_data, 2, axis=1)
        features = np.delete(features, 1, axis=1)

        targets = normalized_data[:,2]
        delays = normalized_data[:, 0]
        predictions = self.predict(features, self.weights)
        N = len(features)
        # Matrix math
        sq_error = (predictions - targets)**2
        print "Predictions:{}, Targets:{}".format(predictions,targets)
        print "Square error: {}".format(sq_error)
        # Return average squared error among predictions
        return 1.0/(2*N) * sq_error.sum()
        

    def predict(self, features, weights):
        """ Predict the expected values for the features and weights provided """
        
        dot_product = np.dot(features, weights)

        # UNCOMMENT BELOW TO SEE PLOT OF PREDICTIONS AS MODEL RUNS (also must uncomment last two lines of main)
        # plot_frequency = 50
        # if len(self.stored_data[:,0]) % plot_frequency == 0:
        #     legend = str(len(self.stored_data[:,0])) + " Features"
        #     plt.plot(self.stored_data[:,0], dot_product, label=legend)

        return np.dot(features,weights)



    def adjust_weights(self):
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
        normalized_data = self.stored_data
        # normalized_data = self.normalize()
        delays = normalized_data[:, 0]
        # correlates = self.stored_data[:, 1]
        wavs = normalized_data[:, 1]

        features = np.delete(normalized_data, 2, axis=1)
        features = np.delete(features, 1, axis=1)
        targets = normalized_data[:, 2]
        predictions = self.predict(features, self.weights)

        d_delay_0 = -delays * (targets - predictions)
        # # d_delay_1 = 
        # d_correlate_0 = 
        # # d_correlate_1 = 
        # d_wav_0 = -wavs * (targets - predictions)
        # # d_wav_1 = 
        print('Features: {}\nWeights: {}\nTargets: {}\nPredictions: {}'.format(features, self.weights, targets, predictions))
        # print("d_delay_0:{}, d_wav_0{}".format(d_delay_0, d_wav_0))

        d_delay_mean = np.mean(d_delay_0)

        self.weights[0] -= math.copysign(1, d_delay_mean) * math.log(math.fabs(d_delay_mean))
        # self.weights[1] -= np.mean(d_wav_0)



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

    # UNCOMMENT BELOW AND SECTION IN PREDICT FUNCTION TO SEE PREDICTION CHANGE OVER TIME
    # plt.legend()
    # plt.show()