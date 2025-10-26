
# Setup

create environment:
```
conda env create -f environment.yml
conda activate sfm # structure from motion
```

# to run the 3D reconstruction for images
update the following paths according to your image folder and prefered output directory:
```
IMAGE_DIR   = Path(f"PATH_TO_YOU_IMAGE_FOLDER")
WORK_DIR    = Path("./work")
OUT_DIR     = Path("./outputs")
```

to use your images use `image_to_pcd.py`:

```
python image_to_pcd.py
```


















