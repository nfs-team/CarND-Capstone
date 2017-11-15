
Traffic Light Localizing and Identifying

Justin Watson
11 Nov. 2017
Team Need For Speed, Udacity, cohort Term 3 Aug. 31

# Training on AWS

To train the model on AWS requires a fair bit of setup. So follow each step
closely.

Create an EC2 instance of udacity-carnd-advanced-deep-learning.

Start the EC2 instance that you just made.

You can now connect to the instance via putty or SCP by using the Public DNS
(IPv4). The username is "ubuntu". There is no password for the "ubuntu" account.

Install the repository model/tensorflow repository. Follow these commands:

```
$ cd ~/tensorflow
$ git clone https://github.com/tensorflow/models.git
```

Read this [README](https://github.com/tensorflow/models/tree/master/research/
object_detection). Then go to the link labeled [Installation](https://github.com/
tensorflow/models/blob/master/research/object_detection/g3doc/installation.md).
Go back to your home directory and start the conda environment.

```
$ source ./miniconda/bin/activate root
$ cd ~/tensorflow/models/research
```

Perform all the steps in the file Installation once you are in the conda
environment and in the research directory. Skip the step on installing tensorflow
because the conda environment already has it. Be sure to test the installation.
The steps are at the bottom on the installation file.

Now you need to have images to train with. The images have to have the traffic
light as a bounding box and the traffic light state. Both those reperesent a
label, which will be the output of the neural network. You can make your own
images and annotate them yourself. You could also use the images with
annotations provided by Anthony Sarkis. Anthony Sarkis shares them at this link
https://drive.google.com/file/d/0B-Eiyn-CUQtxdUZWMkFfQzdObUE/view. You can also
get them from me at https://drive.google.com/file/d/18Zvg-qY43T4_MpyqHvEdkXekDZQNZzXR/view?usp=sharing.

Extract the directory "data" inside the ZIP file to the directory
"~/tensorflow/models/research". You should now have a directory
"~/tensorflow/models/research/data/real_training_data" and "~/tensorflow/models/research/data/sim_training_data". Do not use the files real_data.record or
sim_data.record. You have to make your own TFRecord (.record) files. I will tell
you what to do in the next paragraph, but if you want more information about
what you are doing go to https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/preparing_inputs.md.

To make the TFRecord files you have to run the command:

```
$ cd ~/tensorflow/models/research
$ python data_conversion_udacity_sim.py --output_path sim_data.record
$ python data_conversion_udacity_real.py --output_path real_data.record
```

Make sure you are in your conda environment. You can either make your own
TFRecord script or use Vatsal Srivastava's. The scripts can be found at https://github.com/coldKnight/TrafficLight_Detection-TensorFlowAPI. Copy the scripts to
the directory "~/tensorflow/models/research/".

Get the model from https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md. I used the model ssd_inception_v2_
coco_11_06_2017. You can get this at http://download.tensorflow.org/models/object_detection/ssd_inception_v2_coco_2017_11_08.tar.gz.

```
$ cd ~/tensorflow/models/research
$ wget http://download.tensorflow.org/models/object_detection/ssd_inception_v2_coco_2017_11_08.tar.gz
$ tar -xvzf ssd_inception_v2_coco_2017_11_08.tar.gz
```

Now you need a configuration file for the model. The configuration file tells
the training script where to find data and has values for hyper parameters.
You can download the generic configuration files from https://github.com/tensorflow/models/tree/master/research/object_detection/samples/configs. You
can also use Vatsal Srivastava's located here https://github.com/coldKnight/TrafficLight_Detection-TensorFlowAPI/tree/master/config. You want the files
named ssd_inception\-traffic\_udacity\_real.config and ssd\_inception\-traffic\_udacity\_real.config. Put those files in the directory "~/tensorflow/models/
research".

You also need a label file. Information about what a label file is and how it
looks can currently only be found by looking at an example located here
https://github.com/tensorflow/models/tree/master/research/object_detection/data.
You should get Vatsal Srivastava's [label map](https://github.com/coldKnight/TrafficLight_Detection-TensorFlowAPI/blob/master/label_map.pbtxt).
Save it to the file name label_map.pbtxt in the research directory.

Train your model by entering the following command. Make sure you are in your
conda environment.

Simulator Data

```
$ cd ~/tensorflow/models/research
$ python object_detection/train.py --pipeline_config_path=config/ssd_inception-traffic-udacity_sim.config --train_dir=data/sim_training_data/sim_data_capture
```

Real Data

```
$ cd ~/tensorflow/models/research
$ python object_detection/train.py --pipeline_config_path=config/ssd_inception-traffic_udacity_real.config --train_dir=data/real_training_data
```

If you have this problem

```
TypeError: init() got an unexpected keyword argument 'load_instance_masks'TypeError: init() got an unexpected keyword argument 'load_instance_masks'
```

Go to the section in this document called "Error Running Train Command".

Now you need to save the model for inference.

Simulator Data Based Model

```
$ cd ~/tensorflow/models/research
$ python object_detection/export_inference_graph.py --pipeline_config_path=config/ssd_inception-traffic-udacity_sim.config --trained_checkpoint_prefix=data/sim_training_data/sim_data_capture/model.ckpt-5000 --output_directory=frozen_models/frozen_sim_inception
```

Real Data Based Model

```
$ cd ~/tensorflow/models/research
$ python object_detection/export_inference_graph.py --pipeline_config_path=config/ssd_inception-traffic_udacity_real.config --trained_checkpoint_prefix=data/real_training_data/model.ckpt-10000 --output_directory=frozen_models/frozen_real_inception/
```

Download the directory "frozen_models", which has frozen_real_inception and
frozen_sim_inception. These will be used in your local Jupyter Notebook to
test the models.

# Error Running Train Command

If you have a problem training then you will need to go to a "tensorflow/models"
commit that didn't have a bug. I found that the commit "28b624232f092c15a7fcd9469d0f7327f3a46152" worked. To make a
branch and checkout enter the following.

```
$ git checkout -b traffic 28b624232f092c15a7fcd9469d0f7327f3a46152
```

You will **have to recompile** your protobuf files.

```
# From tensorflow/models/research/
protoc object_detection/protos/*.proto --python_out=.
```

Code above is from the [installation.md](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/installation.md) file.

This bug was filed here https://github.com/tensorflow/models/issues/2634.

# Running Locally to Test the Frozen Model

After you have exported the frozen models for inference comes testing it. To
test it you need to create a local environment and run the jupyter notebook
application.

Create the conda environment locally. Download and install miniconda. Then
create the environment from https://github.com/udacity/CarND-Term1-Starter-Kit.
The file environment.yml contains the environment. You should update the
tensorflow version to the same that was on your AWS EC2 instance. To create
the environment file enter the following command:

```
$ ~/miniconda/bin/conda env create -f environment.yml
```

Start your conda environment. You need to do everything else within the conda
environment.

Clone the tensorflow repository.

```
$ cd ~/src
$ git clone https://github.com/tensorflow/tensorflow.git
```

Clone the repository "tensorflow/models"

```
$ cd ~/src/tensorflow
$ git clone https://github.com/tensorflow/tensorflow.git
```

Run the steps listed in the README file at
"~/src/tensorflow/models/research/object_detection/g3doc/installation.md".
Ensure you ran the test portion of the installation steps.

Place the directory "frozen_models", created during training, into the
directory "~/src/tensorflow/models/research".

Copy the script https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb or get mine from https://drive.google.com/drive/folders/0Bz_YqI8ZbWMxcEhSMzgyalV5VUU?usp=sharing.
If you get the template notebook from the object detection repository you will
need to modify it. You are also going to need some sample images to test the model.
Start "jupyter notebook" application from the research directory.

# References

* https://github.com/coldKnight/TrafficLight_Detection-TensorFlowAPI
* https://github.com/tensorflow/models/tree/master/research/object_detection
