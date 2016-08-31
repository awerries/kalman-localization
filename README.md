# kalman-localization
Implementation of localization using sensor fusion of GPS/INS/compass through an error-state Kalman filter.

The MATLAB code borrows heavily from Paul D. Groves' book, *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*, his MATLAB code is marked as his, and is held under the BSD license. 

This code is very much a work-in-progress as I transition from a MATLAB implementation to C++. Until the code is in a stable state, I will make changes based on my own needs, with complete disregard for backwards compatibility. If you liked it, then you shoulda used a fork on github. Following the initial development, I may make it more portable.

PLEASE NOTE: I cannot provide my testing data as it was collected using a vehicle owned by General Motors. Please, refer to the comments in the initialization scripts for a description of the data format and you should be able to adapt your own data. Alternatively, the book I used comes with a CD-ROM that has his code (fully working filter) along with a system for generating test data.
