# Very Low Budget 3D Scanner

The low budget scanner leveredges an so 101 robot arm, a rotating platform and a 720p webcam, the goal was to deliver an extremely budget friendly robot arm capable of creating a 3D scan of an object.

<p>
<img src="https://imgs.search.brave.com/49AMLh15ySV4qQGzYBF2j14l1QSu9lGU3sD2xOPHRd4/rs:fit:860:0:0:0/g:ce/aHR0cHM6Ly9pLmVi/YXlpbWcuY29tL2lt/YWdlcy9nL1c2Y0FB/ZVN3UHR4b2FLR0cv/cy1sMTYwMC5qcGc" width="33%">
<img src="https://imgs.search.brave.com/SUb_jyiefddCSO73-ezcVJZQ8hgPwJYYtXSmngy7pkc/rs:fit:500:0:1:0/g:ce/aHR0cHM6Ly9tLm1l/ZGlhLWFtYXpvbi5j/b20vaW1hZ2VzL0kv/NDFaeGx3bkstU0wu/anBn" width = "33%">
<img src ="https://imgs.search.brave.com/qmls7_gOsbRaFJ1xLgVKM46FLVRGNZyZi7rnmKFg1Ug/rs:fit:500:0:1:0/g:ce/aHR0cHM6Ly93d3cu/cHJvYWltLmNvbS9j/ZG4vc2hvcC9maWxl/cy9Qcm9haW0tVHVy/bnRhYmxlLTM2MC1S/b3RhdGluZy1QbGF0/Zm9ybS13aXRoLUpv/eXN0aWNrLUNvbnRy/b2xsZXItMV83MDB4/NzAwLmpwZz92PTE3/NDIzNjkyNTM" width = "33%">
</p>

## Data processing pipeline

The data processing pipeline works as follows:

1) masking of the dataset, we take the entire dataset and mask it to remove noise and unnecessary data

2) Feature extraction and matching: features are extracted using sift, the maximum features per image is 2000, in between each image we perform feature matching to estimate the pixel distortion and change

3) structure from motion | bundle adjustment: we use the detected features and their distortion values we construct a 3D representation of the object and save the information as a Point Cloud Data (PCD)

<img src="./readme_resources/pp.png" width="70%" style="display: block; margin: 0 auto" >



# Hardware Used


The hardware used is simply an Aduino UNO and a 5v stepper motor


<p >
<img src="https://imgs.search.brave.com/pbdbR1c915m6F5N5aK5XMiX_3yxePU6eFSBhSZYl9Gc/rs:fit:860:0:0:0/g:ce/aHR0cHM6Ly93d3cu/bWFrZXJzdG9yZS5j/b20uYXUvd3AtY29u/dGVudC91cGxvYWRz/LzIwMTUvMDIvRUxF/Qy0yOEJZSjQ4LVVM/TjIwMDMtMDIud2Vi/cA" width="37%">

<img src="https://imgs.search.brave.com/QuZ3elFnxeebqEAGoflfGx6N9KbaEzK88m8HO2s8ajo/rs:fit:860:0:0:0/g:ce/aHR0cHM6Ly91cGxv/YWQud2lraW1lZGlh/Lm9yZy93aWtpcGVk/aWEvY29tbW9ucy9h/L2E2L0FyZHVpbm9f/VW5vXzAwNi5qcGc" width="48%">
</p >


# Wiring

| Arduino | Stepper |
| ------- | ------- |
| D8      | IN1     |
| D9      | IN2     |
| D10     | IN3     |
| D11     | IN4     |
| 5V      | VCC     |


### Color Coded Wiring

In case you want to skip the documentation of the stepper motor feel free to use the following table:

| Arduino | Stepper |
| ------- | ------- |
| D8      | BLUE    |
| D9      | PINK    |
| D10     | YELLOW  |
| D11     | ORANGE  |
| 5V      | RED     |


### Wiring of the platform | Robot arm | webcam

Everything is directly connected to the "master" pc/laptop, reminder to check for the ports everything connected to

# Setup

You need to perform the calibration for the **FOLLOWER** arm from the official documentation: [link](https://huggingface.co/docs/lerobot/so101)


NOTE: the hardware environment contains the lerobot libraries already by installing it you can skip the step from the documentation

<img src="https://imgs.search.brave.com/JvlmeB4FO1fIkTs66teRGHyXYQCGWzcxFQFPYjEhEVE/rs:fit:860:0:0:0/g:ce/aHR0cHM6Ly9maWxl/cy5zZWVlZHN0dWRp/by5jb20vd2lraS9y/b2JvdGljcy9wcm9q/ZWN0cy9sZXJvYm90/L3NvMTAxL2ZvbGxv/d2VyX21pZGRsZS53/ZWJw" width="60%" style="display: block; margin: 0 auto" >



# Hardware Setup

Anaconda has been used as the environment management tool of choice:

<br>

<img src= "https://imgs.search.brave.com/IVmoslIbtlPZP06T84bhc8hYM41qVP4VECaxP2wz88E/rs:fit:0:180:1:0/g:ce/aHR0cHM6Ly9tZWRp/YS5kZXNpZ25ydXNo/LmNvbS9pbnNwaXJh/dGlvbl9pbWFnZXMv/MTM1NzkwL2NvbnZl/cnNpb25zL18xNTEz/MTA1Nzg4XzExMV9h/bmFjb25kYTJfZTdk/ZmU0NzUzNDg5LWRl/c2t0b3AuanBn" width="50%" style="display: block; margin: 0 auto" >

<br>
<br>

You are expected to mount the webcam/camera of choice to the robot arm 


Ports Used:
arduino: ttyUSB0
arm: ttyACM0

1) setup hardware environmnent:

```
conda env create -f environments/lerobot_env.yml
conda activate sfm # structure from motion
```


# pose recording:
In case you want to record a pose sequence run:
```
python hardware/pose_recording.py
```

- move the robot arm into desired positions
- press "**Enter**" to record the position
- press "**q**" to quit the position recording


# run data recording:

reccomendation run for 1 loop at first to test the system and then increase the number of loops
```
python camera_move_step.py \
  --file hardware/pre_recorded_paths/2pose_data_rec.json \
  --id my_follower \
  --loops 1 \
  --rest_file hardware/pre_recorded_paths/restpose.json \
  --rest_pause_s 1.0
```

## Arduino Code
if you wish to change the angle step you need to update the step degrees function call in the arduino code
```
if (cmd == "+")                  { stepDegrees(10);  Serial.println("OK"); }
else if (cmd == "-")             { stepDegrees(-10); Serial.println("OK"); }
```

# Data Processing 
after recording the data using your robot arm and the rotating platform your can process your recorded dataset using the provided pipeline

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

## Data visualization

The software used to visualize the final point cloud is cloud compare:

<p>
<img src="https://imgs.search.brave.com/5KQvdJ9kI05vT9K55TZa_sDeRa2oCnjoenG6XIasKMI/rs:fit:860:0:0:0/g:ce/aHR0cHM6Ly93d3cu/ZmlsZXBpY2tlci5p/by9hcGkvZmlsZS9R/WlliaVUyc1FnZXR5/enVLVGh2dw" width="33%">

<img src="https://imgs.search.brave.com/vjKixmK5AG8zFrnxUjbsE9tjjbLfz_P8CLl5KfFlIA8/rs:fit:860:0:0:0/g:ce/aHR0cHM6Ly9oZWlz/ZS5jbG91ZGltZy5p/by92Ny9fd3d3LWhl/aXNlLWRlXy9kb3du/bG9hZC9tZWRpYS9j/bG91ZGNvbXBhcmUt/OTE0MDUvY2xvdWRj/b21wYXJlLTFfMS0x/LTE5LmpwZz9mb3Jj/ZV9mb3JtYXQ9YXZp/Zix3ZWJwLGpwZWcm/b3JnX2lmX3NtbD0x/JnE9NzAmd2lkdGg9/MTQ5Ng" width="48%" />

</p>

