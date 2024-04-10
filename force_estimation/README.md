# Deep learning code
This folder contains the code used to train and evaluate our models for the task of force regression from tactile images. In this work, we have used docker to run all the code. I am going to explain a few examples of how to train and test our models.

### 1. Clone the aurova_tactile_sensing repository:
`git clone https://github.com/AUROVA-LAB/aurova_tactile_sensing `

### 2. Build and run the docker image:
`cd aurova_tactile_sensing/force_estimation/`

`docker build -t force_estimation Dockerfile`

`docker run --net=host --gpus "device=0" --rm -it --name force_estimation -v ./:/deepl_force force_estimation`

### 3. Training example
- The 'code' folder contains all the scripts to train and evaluate our model for the task of force regression from tactile images. We provide a trained model to estimate forces from images with the DIGIT sensor, and also a small version of our dataset only to test that our code works. The full dataset will be available soon too.
- Go to code folder inside of the docker container:
`cd ../deepl_force/code/`
- Train the RGBmod model, saving the weights into a folder with a scecific name:
- `python train.py --train_mode 0  --save True --training training1`

### 4. Test example
- Test our trained model with a few images from the test_data folder:
- `python test.py --train_mode 0`

# Force estimation ROS package
This package contains one script to run the force regression model inside of ROS. This script subscribes to the DIGIT tactile sensor topic, estimates the force, and publish it in another topic. The instructions are described in the applications repository.

