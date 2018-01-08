# A Novel Wearable Device to Alleviate Freezing of Gait in Parkinson's Patients

This repository contains the algorithm for a device that detects freezing of gait in Parkinson's patients through MPU6050 data. It separates gait into individual steps, calculates features from the steps, then uses a linear support vector machine (SVM) to classify each step.

[1 page project summary (with pictures!)](https://drive.google.com/file/d/1soIrgk60Apn2dH2Q9rNKHtnACKmGRsOp/view)

## Getting Started
### Requirements
#### Software
* Install the latest version of the MPU6050 library, found [here](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)

### Included Files
This repository contains three iterations of the code: 
* "firstAlgo" was a quick algorithm that classified the type of gait based on minimally processed accelerometer values. 
* Main contain the mostly complete iteration that utilizes gait segregation and a binary SVM. 
* Bluetooth Codebase was used for the final prototype. The RedBearLab BLE Nano microcontroller was for its bluetooth functionalities. 
* The two csv files contain sample angle data for healthy and freezing gait. 
* SVM contains matlab code to run the support vector machine and find optimized thresholds. 


## Instructions

The following instructions are for using the SVM to find optimized thresholds for the two calculated features: 

1. Run Main.ino and gather at least 50 data points. 
2. Save the data points as a csv file
3. Manually label each data point in a new column in that same csv file
4. Load the csv file in Matlab.
5. Download and navigate to the three included matlab files
6. Run the following command in Matlab: 
```
SVM(X,y,C,max_iter)
```
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; X is an array containing the first two columns of the csv file <br/>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; y contains the 3rd column of the csv file<br/>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; C = 1 (A larger value would lead to greater consideration of outliers)<br/>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; max_iter = 20<br/><br/>

7. In Main.ino lines 64 and 65, replace the values of w and b with the corresponding values returned in step 6. 


## Authors

Andrew Luo <br/>
Collete McKee<br/>
Howard Yu<br/>
Alexander Tummon Simmons<br/>
Nathan Duarte
