from utils import get_test_loader, load_checkpoint
from models import RGBmod, D, RGBandDToForce
import torch
import torch.nn as nn
import torchvision.transforms as T

import argparse
import os
from tqdm import tqdm
import numpy as np
import time


# define the path to load the weights of your model
load_path = "./weights/trained_model_digit"

# parameters 
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
# value of 1 for debugging
BATCH_SIZE = 1
NUM_WORKERS = os.cpu_count()
PIN_MEMORY = True  # pin_memory speeds up the transfer of the dataset from CPU to GPU during training.


# function to validate the model
def validation_fn(loader, model, loss_fn, train_mode):

	model.eval()

	loop = tqdm(loader)

	running_loss = []
	test_loss_list = []
	times = []
	normalized_error = []

	with torch.inference_mode():

		for img, img_depth, f in loop:

			# measure t1
			t1 = time.time()

			# 0 - RGBmod, 1 - D, 2 - RGBmod+D

			if train_mode == '0':
				img, f = img.to(DEVICE), f.float().unsqueeze(1).to(DEVICE)
				predictions = model(img)	

			elif train_mode == '1':
				img_depth, f = img_depth.to(DEVICE), f.float().unsqueeze(1).to(DEVICE)
				predictions = model(img_depth)

			else:
				img, img_depth, f = img.to(DEVICE), img_depth.to(DEVICE), f.float().unsqueeze(1).to(DEVICE)	
				predictions = model(img, img_depth)


			# measure t2
			t2 = time.time()
			#print(t2-t1)
			times.append(t2-t1)		

			#print(predictions, f)
			e = torch.abs(predictions-f) / f
			#print(e)
			normalized_error.append(e.item())

			# loss
			loss = loss_fn(predictions, f)
			running_loss.append(loss.item())
			test_loss_list.append(loss.item())

			loop.set_postfix(loss=loss.item())

	return np.mean(running_loss), test_loss_list, np.array(times), normalized_error



def main(args):

	test_transform = T.Compose(
		[
		#A.Resize(height=IMAGE_HEIGHT, width=IMAGE_WIDTH),
		T.ToTensor(),
		T.Normalize(
			mean=(0.0),
			std=(1.0),
		),
		],
	)

	test_loader = get_test_loader(
		TEST_IMG_DIR,
		TEST_IMG_DEPTH_DIR,
		TEST_F_DIR,
		test_transform,
		BATCH_SIZE,
		args.train_mode,
		NUM_WORKERS,
		PIN_MEMORY
	)

	# define nn model
	model = None

	# 0 - RGBmod, 1 - D, 2 - RGBmod+D
	if args.train_mode == '0':
		model = RGBmod().to(DEVICE)
	elif args.train_mode == '1':
		model = D().model.to(DEVICE)
	else:
		model = RGBandDToForce().to(DEVICE)

	load_checkpoint(torch.load(os.path.join(load_path, "checkpoint_epoch_24.pth.tar")), model)

	loss = nn.L1Loss()

	test_loss, test_loss_list, inf_time, normalized_error = validation_fn(test_loader, model, loss, args.train_mode)
	print(f"Normalized error with respect to the force magnitude: {np.mean(normalized_error)} +- {np.std(normalized_error)}")

	test_std = np.std(test_loss_list)
	print(f"Test loss: {test_loss} +- {test_std}; Inference time: {np.mean(inf_time)} \n")



if __name__ == '__main__':
	
	parser = argparse.ArgumentParser()
	parser.add_argument("-m", "--train_mode", required=True, type = str, help = 'Training mode (nn input)')
	args = parser.parse_args()

	#TODO: change dataset path to load your own images
	dataset_path = "./test_data"

	TEST_IMG_DIR = f"{dataset_path}/images"
	TEST_IMG_DEPTH_DIR = f"{dataset_path}/filt_img_depth"
	TEST_F_DIR = f"{dataset_path}/forces"
	
	print(TEST_F_DIR)
	
	main(args)
