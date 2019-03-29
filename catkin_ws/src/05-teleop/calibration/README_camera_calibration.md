# Camera Calibration Verification Test {#demo-camcaltest status=beta}

This document provides instructions for testing the camera calibration. 


<div class='requirements' markdown="1">

Requires: A duckiebot version DB-18.

Requires: USB drive.

Requires: Calibration calibration test hardware, see below.
</div>


## Pre-flight checklist {#demo-camcaltest-pre-flight}

Check: the USB plugged in.

Check: the duckiebot has sufficient battery

## Abbreviations {#demo-camcaltest-abb}

`DOCKER_CONTAINER]` = duckietown/rpi-duckiebot-base:devel-optimization_v1

`SOFTWARE_ROOT` = /home/software

`PACKAGE_ROOT` = `SOFTWARE_ROOT`/catkin_ws/src/05_teleop/calibration

## Demo instructions {#demo-camcaltest-run}


**Step 1**: SSH into your duckiebot and create the directory for logging.

    duckiebot $ sudo mkdir /data/logs

**Step 2**: Then mount your USB

    duckiebot $ sudo mount -t vfat /dev/sda1 /data/logs -o umask=000

**Step 3**: Now we will run the docker container on our duckiebot that contains the test script. Open a new terminal on your computer. Make sure that `DOCKER_HOST` variable is set by checking

    laptop $ echo $DOCKER_HOST

if the output is empty then `DOCKER_HOST` is not set. You can set it with

    laptop $ export DOCKER_HOST=![ROBOT_NAME].local

Now, we will run the docker container. Be sure to replace the `DOCKER_CONTAINER` with the name provided under Abbreviations section.

    laptop $ docker -H ![HOST_NAME].local run -it --net host --privileged -v /data/logs:/logs -v /data:/data --memory="800m" --memory-swap="2.8g" --name camera-test ![DOCKER_CONTAINER] /bin/bash

Depending on you network speed it might take some time until the duckiebot downloads the container.

**Stage 4**: Now Place your duckiebot inside the camera calibration hardware.

Having the experimental setup ready, we can start testing, enter into the running `camera-test ` container if your are not already in, and launch the test interface.

    laptop $ export DOCKER_HOST=![ROBOT_NAME].local

    laptop $ docker exec -it sysid-pi /bin/bash

    duckiebot $ roslaunch calibration camera_calibration_test.launch veh:=![HOST_NAME] output_rosbag_dir:=/logs
    

Note that, if `output_rosbag_dir` is not specified the program attempts to save the results to user´s home folder. This will fail it you don't have enough space in your device.

With data-acquisition interface you can specify

* the type of the experiment you would like to conduct by choosing amongst the presented options,

* whether to save the collected experiment data by replying to the question after the experiment has been completed,

* whether to do another experiment.


The results of the experiment can be found under `logs` folder in a zipped form. To download the results ![ZIPPED_RESULT_NAME] to your local computer,
    
    duckiebot $ mv ...



## Troubleshooting {#demo-camcaltest-troubleshooting}

Symptom: No log have been recorded.

Resolution: Make sure you mounted USB drive. Please note that you have to should first mount it correctly before you can start data collection.

Symptom: Logs are created but they are empty.

Resolution: This might be because the Raspberry-Pi did not have enough time to save the data. Please increase `wait_start_rosbag` and `wait_write_rosbag` inside [this script](https://github.com/selcukercan/Software/blob/system-identificiation-v1/catkin_ws/src/05-teleop/calibration/src/data_collector.py).

Symptom: The duckiebot deviates from the trajectory, so that the AprilTag goes out of the camera’s field of view.

Resolution: You can adjust the parameters of each command to maximize the duration

Symptom: There are large discontinuities in the recordings despite the fact that the duckiebot does see the AprilTag most of the time.

Resolution: One possible cause of this problem is insufficient memory. Please make sure to execute `docker run` command with `--memory="800m" --memory-swap="2.8g"` flags which would tell docker to utilize the swap space. Swap space is created and allocated during the initialization process. The swap space allocation is done by default since 5 October 2018. If you had flashed your SD card prior to that, please reflash your SD card. You can verify that you have swap space by executing `top` command in your duckiebot and inspecting `KiB Swap` section.

Symptom: My problem is not listed here. How do I get help?

Resolution: Though we tested the system identification procedure multiple times on different duckiebots, it is possible that something did not work for you. Please file an issue on GitHub, [here](https://github.com/selcukercan/Software/issues).

## Development Notes

If you would like to develop locally on your computer you should record a rosbag and use it to emulate real operation. 

After taking a rosbag containing the topics necessary for your application, on your computer start a roscore, start playing the rosbag you recorded in the background and then execute


    laptop $ roslaunch calibration camera_calibration_test_node.launch veh:=![HOST_NAME] output_dir:=![OUTPUT_DIR]
    
   