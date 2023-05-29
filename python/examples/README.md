# How to

This directory contains the original examples of using the python API for basic tasks in the simulator.
Additionally we provided some scripts for use cases that are helpful for us.

Note: while these scripts aim to be applicable for general use cases, we did design them for our needs and as such they might contain dirs,
or outside entities such as a buckets and other endpoints which are inaccessible to the public. 

We use the setup in `/docker` to run the sim.

## How to generate Bounding Boxes Directly from the sim.

This script, together with services we added to the fsds packages drives around the track and records images and their segmenation masks.
The script then uploads the images to a gcloud bucket, where a cloud function processes the masks into bounding boxes
