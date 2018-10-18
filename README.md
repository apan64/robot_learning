# robot_learning
Team members: Andrew Pan, Robbie Siegel

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

