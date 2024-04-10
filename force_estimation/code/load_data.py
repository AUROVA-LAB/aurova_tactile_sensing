# class to load our dataset
import os
from PIL import Image
from torch.utils.data import Dataset
import numpy as np
import cv2 


class VisionToForceDataset(Dataset):

  def __init__(self, image_dir=None, img_depth_dir=None, force_dir=None, transform=None, train_mode=None):

    self.image_dir = image_dir
    self.images = os.listdir(image_dir)

    self.image_depth_dir = img_depth_dir
    self.images_depth = os.listdir(img_depth_dir)

    self.force_dir = force_dir
    self.forces = os.listdir(force_dir)

    # 0 - RGBmod, 1 - D, 2 - RGBmod+D
    self.train_mode = train_mode
    self.transform = transform

    
  def __len__(self):
    return len(self.images)


  def __getitem__(self, index):

    image = []
    img_depth = []
    f = []

    # load the path of images and forces
    # 0 - RGBmod, 1 - D, 2 - RGBmod+D
    if self.train_mode == '0':
      img_path = os.path.join(self.image_dir, self.images[index])
      image = np.array(Image.open(img_path).convert("RGB"))

    elif self.train_mode == '1':
      img_depth_path = os.path.join(self.image_depth_dir, self.images_depth[index])
      img_depth = cv2.imread(img_depth_path, 0)
      
    else:
      img_path = os.path.join(self.image_dir, self.images[index])
      image = np.array(Image.open(img_path).convert("RGB"))
      img_depth_path = os.path.join(self.image_depth_dir, self.images_depth[index])
      img_depth = cv2.imread(img_depth_path, 0)

   
    f_path = os.path.join(self.force_dir, "force_" + self.images[index]).replace(".png", ".txt")
   
    # load corresponding force value for this tactile image
    with open(f_path, "r") as f:
        f = f.read()
        f = float(f.split("\n")[0])
  

    # apply transformations
    # 0 - RGBmod, 1 - D, 2 - RGBmod+D
    if self.transform is not None:
    
        if self.train_mode == '0':
          image = self.transform(image)

        elif self.train_mode == '1':
          img_depth = self.transform(img_depth)
        
        else:
          image = self.transform(image)
          img_depth = self.transform(img_depth)
          

    return image, img_depth, f


    