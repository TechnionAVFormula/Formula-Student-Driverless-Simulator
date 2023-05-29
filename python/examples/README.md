# How to

This directory contains the original examples of using the python API for basic tasks in the simulator.
Additionally we provided some scripts for use cases that are helpful for us.

Note: while these scripts aim to be applicable for general use cases, we did design them for our needs and as such they might contain dirs,
or outside entities such as a buckets and other endpoints which are inaccessible to the public. 

We use the setup in `/docker` to run the sim.

## 1. How to generate Bounding Boxes Directly from the sim.

This script, together with services we added to the fsds packages drives around the track and records images and their segmenation masks.
The script then uploads the images to a gcloud bucket, where the masks are processed into bounding boxes.

### 1. Run the simulator with image_recorder:
```
./setup.sh image_recorder
```


### 2. Run the image recording script:
```
cd Formula-Student-Driverless-Simulator/python/example
python3 image_recorder.py
```
Note that if this is your first time running this you might get errors about the `google_application_credentials` env variable. Simply download your service account's keys and put the in the required dir.


### 3. The script should upload the images and masks to the bucket.

Note that if you want to add additional, I/O intensive or blocking functionality to the script, make sure to add it on the existing secondary threads or create a new one. 
If you add the code on the main thread (i.e. the one that drives the car) you can potentially crash it or cause it to abruptly go off-track.


