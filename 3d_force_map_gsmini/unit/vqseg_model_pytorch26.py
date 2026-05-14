import hydra
from omegaconf import OmegaConf
import pathlib

import torch
import torch.nn as nn
import numpy as np

from vqgan_pytorch26 import VQModel
from base_workspace.base_workspace import BaseWorkspace

import random


class VQVAE(nn.Module):

    def __init__(self, cfg, model_path):
        super().__init__()

        seed = cfg.seed
        torch.manual_seed(seed)
        np.random.seed(seed)
        random.seed(seed)

        self.model = VQModel(**cfg.model)
        ckptdir = model_path
        #ckptdir = "./weights/markers_checkpoint-epoch=340.ckpt"
        self.model.init_from_ckpt(ckptdir)
        self.enc = self.model.encoder


    def forward(self, x):
        return self.model(x)





@hydra.main(
    version_base=None,
    config_path = "./config/",
    config_name = "config"
)
def main(cfg):
    
    vqvae_model = VQVAE(cfg)


if __name__ == '__main__':
    main()
