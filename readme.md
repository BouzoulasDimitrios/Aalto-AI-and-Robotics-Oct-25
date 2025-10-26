# Very Low Budget 3D Scanner

The low budget scanner leveredges an so 101 robot arm and a 720p webcam, the goal was to deliver an extremely budger friendly robot arm capable of creating a 3D scan of an object.

<p>
<img src="https://imgs.search.brave.com/49AMLh15ySV4qQGzYBF2j14l1QSu9lGU3sD2xOPHRd4/rs:fit:860:0:0:0/g:ce/aHR0cHM6Ly9pLmVi/YXlpbWcuY29tL2lt/YWdlcy9nL1c2Y0FB/ZVN3UHR4b2FLR0cv/cy1sMTYwMC5qcGc" width="48%">
<img src="https://imgs.search.brave.com/SUb_jyiefddCSO73-ezcVJZQ8hgPwJYYtXSmngy7pkc/rs:fit:500:0:1:0/g:ce/aHR0cHM6Ly9tLm1l/ZGlhLWFtYXpvbi5j/b20vaW1hZ2VzL0kv/NDFaeGx3bkstU0wu/anBn" width = "48%">
</p>

## Data processing pipeline


<img src="./readme_resources/pp.png" width="70%">



# Hardware Used


The hardware used is simply an arduino uno and a 5v stepper motor


<p>
<img src="https://imgs.search.brave.com/pbdbR1c915m6F5N5aK5XMiX_3yxePU6eFSBhSZYl9Gc/rs:fit:860:0:0:0/g:ce/aHR0cHM6Ly93d3cu/bWFrZXJzdG9yZS5j/b20uYXUvd3AtY29u/dGVudC91cGxvYWRz/LzIwMTUvMDIvRUxF/Qy0yOEJZSjQ4LVVM/TjIwMDMtMDIud2Vi/cA" width="37%">

<img src="https://imgs.search.brave.com/QuZ3elFnxeebqEAGoflfGx6N9KbaEzK88m8HO2s8ajo/rs:fit:860:0:0:0/g:ce/aHR0cHM6Ly91cGxv/YWQud2lraW1lZGlh/Lm9yZy93aWtpcGVk/aWEvY29tbW9ucy9h/L2E2L0FyZHVpbm9f/VW5vXzAwNi5qcGc" width="48%">
</p>


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





# Setup

# Hardware Setup

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
  --file /home/dimitrios/git_repos/robotics_hackathon/demo_scripts/state/captures/peji01.json \
  --id my_follower \
  --loops 1 \
  --rest_file /home/dimitrios/git_repos/robotics_hackathon/demo_scripts/state/captures/restpose.json \
  --rest_pause_s 1.0
```

## Arduino Code
if you wish to change the angle step  

```

```

# Data Processing 
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



