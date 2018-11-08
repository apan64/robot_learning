# robot_learning
Team members: Andrew Pan, Robbie Siegel

## Writeup
In this project, we aimed to implement a learning algorithm from the ground-up enabling a robot to search for a snapping sound and identify direction of its origin, then turn towards the point and begin moving forwards.  

We collected learning set data using the DataGenerator class located in data_collection.py. The set up for data collection with this class involved hooking up a left and right microphone to the front of the Neato and placing a speaker about five feet away, directly in front of it. This class would cycle through two steps: playing an audio recording from a speaker while simultaneously recording audio data from the Neato microphones, and then rotating the neato about 6°. The audio recordings would switch between four audio files, three of which were of snaps with background noise, while one was of just background noise. This script allowed us to collect data for all four audio recordings at a variety of angles spaced around the 360° the Neato could turn relative to the speaker playing audio. Though we recorded data from -180° to 180°, we only used those recorded between -90° and 90° in our final implementation since the offset between the left and right microphones was very similar between recordings collected at angles 180° apart. 

There were a couple of points over the course of the project in which we had to make design decisions in order to move forward, one of which being that we had to determine how we actually wanted to implement our learning algorithm.  Initially we wanted to implement two separate learning algorithms: one for the audio localization and one for detecting a snap from background noise. We decided that we would begin with implementing linear regression for the localization as first experience in learning algorithms, but we soon ran into several roadblocks that caused us to switch our entire project over to linear regression in order to complete it on time.  Unfortunately, our algorithm failed to actually obtain the correct weights to match the slope of our data.  We tried experimenting with various types of normalization and different features to feed into the algorithm, but most attempts did not produce any meaningful or useful results.

Given more time, we would have continued to investigate the issues behind our learning algorithm, as we were thoroughly confused as to what was going wrong, especially because at some point we more or less followed instructions from a source that we found online.  We also would have liked to try investigating polynomial regression, since our data did not seem strictly linear as the angles approached larger magnitudes.  Finally, we would have experimented with different methods of determining what qualifies as a snap in an audio file, as we didn’t have enough time to properly work with different analyses of the WAV files.

Although the final outcome of our project was not as successful as we had hoped, it was a valuable experience investigating a variety of learning algorithms at a high level and then gaining a deeper understanding of our chosen approach, linear regression. It was very important for us to understand the math behind each part of our implementation so that we could not only implement it in the best way, but also debug when our results were off. It was also helpful for us to visualize our model’s results so that we could see in what ways it was improving (or failing to improve) throughout the learning process. 


## Proposal
### Summary
In this project, we aim to develop a learning algorithm using a bottom-up approach to implement said learning algorithm ourselves.  We plan to train our algorithm on a set of audio recordings in order to have a robot identify the source of a snapping sound and move towards the source.  This will involve a fair amount of work in the field of audio processing, primarily in frequency separation and localization based on the data available to us from the robot’s hardware.  Because we intend to work with a bottom-up approach, we will also commit a significant portion of our work towards researching and identifying an appropriate learning algorithm to use for audio processing, and make any necessary tweaks to best suit our use case.

Because we would like our focus to be on developing and understanding a bottom-up approach to solving this problem, our MVP will be an algorithm that is able to accurately detect snapping sounds ~75% of the time. 

### Blog posts:
[https://www.analyticsvidhya.com/blog/2018/01/10-audio-processing-projects-applications/](https://www.analyticsvidhya.com/blog/2018/01/10-audio-processing-projects-applications/)
[https://www.analyticsvidhya.com/blog/2017/08/audio-voice-processing-deep-learning/](https://www.analyticsvidhya.com/blog/2017/08/audio-voice-processing-deep-learning/)

### Data Collection
We will record many audio clips of snapping. We will record ourselves snapping, and we will also recruit other students to let us record them snapping. We will snap using different fingers one each of our hands to obtain a wide variety of snapping volumes, frequencies, and lengths. 

In terms of learning algorithms, we may start with implementing a very basic linear regression-based algorithm to gain a better understanding of learning algorithm development initially, then utilize the lessons learned from that implementation in selecting a more appropriate algorithm for further usage in this project.  For baseline comparison, we will be comparing the performance of our learning algorithm to a naive implementation of audio localization based on relative volume detected by the robot.

