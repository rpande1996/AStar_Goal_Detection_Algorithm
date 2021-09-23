# AStar goal detection of a rigid robot:

# Code developed by:

Rajan Pande: rpande@umd.edu

Abhilash Mane: amane@umd.edu

This repository takes in user input about start point end point orientation and calculates A* path using video of exploration and backttracking gets saved in folder
This code has been runned on 32 gb Ram pc
The code is done on Anaconda Spyder on Python 3.7 and recommended to perform on the same if possible

Libraries and command to install those:

    Virtual Evirobnment with Python 3.7 : conda create -n myenv python=3.7 
    Numpy : conda install -c anaconda numpy
    Matplotlib -: conda install -c conda-forge matplotlib 
    Imutils - conda install -c conda-forge imutils

Instruction to run the code:

    1. Create a Virtual Environment for python 3.7
        conda create -n myenv python=3.7
    2. Activate the Virtual Environment
        conda activate myenv 
    3. Install the Libraries and dependency
        conda install -c anaconda numpy
        conda install -c conda-forge matplotlib 
	conda install -c conda-forge opencv=4.0.1
        conda install -c conda-forge imutils
    4. Make sure the all files are in same folder and run the abhilashmane_rajanpande.py
    5. The boundaries are given from 0-299 and 0-399 respectively for y and x 
    Enter the following input coordinates
        
         Start coordinates 

           x_coordinate : 

           y_coordinate : 

          theta_start : 
        Goal coordinates 

          x_coordinate :

          y_coordinate :

          theta_goal : 
        Other parameters 

           step size : 

           radius size : 

           clearance size : 
