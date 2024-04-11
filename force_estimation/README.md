# Deep learning code
This folder contains the code used to train and evaluate our models for the task of force regression from tactile images. 

### 1. Clone the aurova_tactile_sensing repository:
`git clone https://github.com/AUROVA-LAB/aurova_tactile_sensing `

### 2. Set up the conda environment:
`cd aurova_tactile_sensing/force_estimation/`

`conda create --name==digit_ros_torch python=3.8.10`

`conda activate digit_ros_torch`

`pip install rospkg empy`

`pip install numpy`

`pip install opencv-python`

`pip install digit-interface`

`conda install pytorch==1.12.0 torchvision==0.13.0 torchaudio==0.12.0 cudatoolkit=11.3 -c pytorch`

`pip install tqdm`

`pip install wandb`

### 3. Training example
- The 'code' folder contains all the scripts to train and evaluate our model for the task of force regression from tactile images. We provide a trained model to estimate forces from images with the DIGIT sensor, and also a small version of our dataset only to test that our code works. The full dataset will be available soon too.
- Train the RGBmod model, saving the weights into a folder with a scecific name:
- `cd code/`
- `python train.py --train_mode 0  --save True --training training1`

### 4. Test example
- Test our trained model with a few images from the test_data folder:
- `python test.py --train_mode 0`

# Force estimation ROS package
This package contains one script to run the force regression model inside of ROS. This script subscribes to the DIGIT tactile sensor topic, estimates the force, and publish it in another topic. The instructions are described in the applications repository [applications](https://github.com/AUROVA-LAB/applications/tree/main/app_force_estimation).

