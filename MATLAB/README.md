# MATLAB Implementation of Adaptive Kalman Filter
The MATLAB code borrows heavily from Paul D. Groves' book, *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*, his MATLAB code is marked as his, and is held under the BSD license. Edits to his files are marked in comments either on the line, or above a section. Code that was left un-edited has been placed into the GrovesCode folder. Purely my code is labeled as such at the top.

Files in the Utilities folder are also not my code, but rather code found through MathWorks File Exchange.

# Running the code
Simply run *initialize\_and\_run.m*, and it will prompt you to select a ground truth file (Applanix format), a NovAtel log (pre-parsed by my *read\_gps\_log.py* code in awerries/rpi-sensors/Python_Tools), and the IMU log (pre-parsed by my *read\_imu\_log.py* in the same place). I may make some sample data available in the future.
