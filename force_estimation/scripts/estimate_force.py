#!/home/aurova/anaconda3/envs/digit_ros_torch/bin/python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

import ros_numpy

import cv2
import os

import torch
import torchvision.transforms as T

import sys
sys.path.append("../code/")

from utils import load_checkpoint
from models import RGBmod


DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
BATCH_SIZE = 1
NUM_WORKERS = os.cpu_count()
PIN_MEMORY = True

digit_image = None


def publish_force_value(digit_image, model, test_transform, pub_force):
	'''
	Publish the estimated force value in N from the RGB image.
	'''

	if digit_image is not None:

		frame = digit_image
		frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

		preprocess_img = test_transform(frame_rgb)

		preprocess_img_cuda = preprocess_img.unsqueeze(0).to(DEVICE)
		predicted_force = model(preprocess_img_cuda) 

		post_pred_force = predicted_force.detach().cpu().numpy().squeeze()
		
		pub_force.publish(post_pred_force)



def callback(img, args):
	
	global digit_image

	model = args[0]
	test_transform = args[1]
	pub_force = args[2]

	digit_image = ros_numpy.numpify(img)

	publish_force_value(digit_image, model, test_transform, pub_force)


def main():

	rospy.init_node('force_estimation', anonymous=True)

	pub_force = rospy.Publisher(f"digit/force_value", Float32, queue_size=10)

	load_path = "../code/weights/trained_model_digit/checkpoint_epoch_24.pth.tar"

	model = RGBmod().to(DEVICE)

	load_checkpoint(torch.load(os.path.join(load_path)), model)

	model.eval()

	test_transform = T.Compose(
		[
		T.ToTensor(),
		T.Normalize(
			mean=[0.0, 0.0, 0.0],
			std=[1.0, 1.0, 1.0],
		),
		],
	)

	digit_img_sub = rospy.Subscriber("digit55/camera/image_color", Image, callback, (model, test_transform, pub_force))

	rospy.spin()
	
	

if __name__ == "__main__":
	rospy.loginfo("starting...")
	main()
