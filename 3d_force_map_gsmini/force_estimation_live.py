import sys
import torch
import cv2
import numpy as np
import hydra
import gsdevice

# Add paths
sys.path.append("./unit")
sys.path.append("./force")
from vqseg_model_pytorch26 import VQVAE
from models.unet import UNet
from data.dataloader import unnormalize

# --- CONFIG ---
CHECKPOINT_VQ = "./unit/weights/vq_gan_mid/checkpoint-epoch=34.ckpt"
H, W = 128, 160
TARGET_SIZE = (320, 240)  # (Width, Height) for cv2.resize
WARM_UP_ITERS = 10

# --- HELPER FUNCTIONS ---

def setup_windows(win_names, width=400, height=300):
    """Initializes OpenCV windows once."""
    for name in win_names:
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(name, width, height)

def capture_single_model_graph(model, input_shape):
    """Records a CUDA Graph for static inference speed."""
    print(f"Capturing CUDA Graph for {type(model).__name__}...")
    static_input = torch.zeros(input_shape, dtype=torch.float32, device='cuda')
    
    # Warmup
    s = torch.cuda.Stream()
    s.wait_stream(torch.cuda.current_stream())
    with torch.cuda.stream(s), torch.inference_mode():
        for _ in range(WARM_UP_ITERS):
            _ = model(static_input)
    torch.cuda.current_stream().wait_stream(s)
    
    # Capture
    g = torch.cuda.CUDAGraph()
    torch.cuda.synchronize()
    with torch.cuda.graph(g), torch.inference_mode():
        static_output = model(static_input)
    
    return g, static_input, static_output

def post_process_force(pred_channel, mode, cmin, cmax, cfg):
    """Unnormalizes, colorizes, and resizes a single force channel."""
    # 1. Unnormalize and clip
    val = unnormalize(pred_channel, mode, cfg["norm_file"])
    val = np.clip(val.copy(), cmin, cmax)
    
    # 2. Scale to 0-255 and apply colormap
    norm = ((val - cmin) / (cmax - cmin) * 255).astype(np.uint8)
    colored = cv2.applyColorMap(norm, cv2.COLORMAP_VIRIDIS)
    
    # 3. Resize for consistent display
    return cv2.resize(colored, (400, 300), interpolation=cv2.INTER_NEAREST), np.sum(val)

# --- CORE LOGIC ---

@hydra.main(version_base=None, config_path="./config/", config_name="config")
def fast_inference(cfg):
    # Constants
    FXY_LIMS = (-0.07, 0.07)
    FZ_LIMS = (-0.3, 0.0)
    
    # 1. Setup Models
    print("Initializing Models...")
    vq_wrapper = VQVAE(cfg, CHECKPOINT_VQ)
    vq_model = vq_wrapper.model.eval().cuda()
    
    force_model = UNet(enc_chs=cfg["enc_chs"], dec_chs=cfg["dec_chs"], out_sz=cfg["output_size"])
    force_model.load_state_dict(torch.load(cfg["force_model"], map_location="cuda", weights_only=True))
    force_model.eval().cuda()

    # 2. Capture CUDA Graphs
    g1, static_in_vq, static_out_vq = capture_single_model_graph(vq_model, (1, 3, H, W))
    g2, static_in_unet, static_out_unet = capture_single_model_graph(force_model, (1, 3, 240, 320))

    # 3. Setup Camera & Background
    dev = gsdevice.Camera("GelSight Mini")
    dev.connect()
    
    bg = cv2.imread("./rec_bg.png")
    # Pre-calculate background embedding to save time in loop
    with torch.no_grad():
        # 1. Resize and normalize background
        bg_resized = cv2.resize(bg.astype(np.float32)/255, (W, H))
        # 2. Convert to (1, 3, H, W)
        bg_tensor = torch.from_numpy(bg_resized).permute(2, 0, 1).unsqueeze(0).cuda()
        # 3. Get VQ Embedding
        bg_raw_emb = vq_model.get_input({"image": bg_tensor}, "image")
        bg_raw_emb = bg_raw_emb.permute(0, 2, 3, 1)
        # 4. Upsample to match UNet input (240, 320)
        bg_emb = torch.nn.functional.interpolate(bg_raw_emb, size=(240, 320), mode='bilinear')
        

    # 4. Initialize UI
    win_names = ["Original", "Force X", "Force Y", "Force Z"]
    setup_windows(win_names)

    upsampler = torch.nn.Upsample(size=(240, 320), mode='bilinear', align_corners=False)

    print("Starting Inference Loop...")
    try:
        while dev.while_condition:
            img = dev.get_image()
            
            # --- PRE-PROCESS ---
            img_in = cv2.resize(img.astype(np.float32)/255, (W, H))
            img_tensor = torch.from_numpy(img_in).permute(2,0,1).unsqueeze(0).cuda()

            # --- VQVAE INFERENCE ---
            static_in_vq.copy_(img_tensor)
            g1.replay()
            vq_out = static_out_vq[0] if isinstance(static_out_vq, (list, tuple)) else static_out_vq
            
            # --- DIFFERENCE CALCULATION ---
            x_inter = upsampler(vq_out)
            diff_tensor = torch.clip((x_inter - bg_emb) + 0.5, 0, 1) 

            # --- UNET INFERENCE ---
            static_in_unet.copy_(diff_tensor)
            g2.replay()
            torch.cuda.synchronize()
            
            # --- POST-PROCESS & VISUALIZE ---
            res = static_out_unet.squeeze(0).permute(1, 2, 0).detach().cpu().numpy()
            
            I_fx, T_fx = post_process_force(res[:,:,0], "grid_x", *FXY_LIMS, cfg)
            I_fy, T_fy = post_process_force(res[:,:,1], "grid_y", *FXY_LIMS, cfg)
            I_fz, T_fz = post_process_force(res[:,:,2], "grid_z", *FZ_LIMS, cfg)
            
            print(f"T_fx = {T_fx}")
            print(f"T_fy = {T_fy}")
            print(f"T_fz = {T_fz}")
            print(f"Total Force = {np.sqrt(T_fx**2 + T_fy**2 + T_fz**2)}")

            cv2.imshow("Original", cv2.resize(img, (400, 300)))
            cv2.imshow("Force X", I_fx)
            cv2.imshow("Force Y", I_fy)
            cv2.imshow("Force Z", I_fz)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print('Interrupted!')
    finally:
        dev.stop_video()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    fast_inference()