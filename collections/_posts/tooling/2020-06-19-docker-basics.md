---
layout: single
title:  "Docker Basics"
date:   2020-06-19 19:31:41 +0200
excerpt: "Docker Container Basics."
permalink: /posts/tooling/docker/
categories: [docker, container, virtualization]
tags: [docker, container, tooling, virtualization, Dockerfile]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Kalman filter"
header:
  teaser: /assets/images/docker-logo.jpg
  overlay_image: /assets/images/docker-logo.jpg #keep it square 200x200 px is good
  overlay_filter: 0.5 # same as adding an opacity of 0.5 to a black background
  caption: "Source: [**t3n**](https://t3n.de/news/docker-hosting-613802/)"
  show_overlay_excerpt: true
---


## Docker Basics

Basic Dockerfile:

```bash
FROM alpine:latest


```


## Basic Workflow


```
#!/bin/bash
export PROJECT_ID=$(gcloud config list project --format "value(core.project)")
export IMAGE_REPO_NAME=my-project-tf2.3-gpu
export IMAGE_TAG=latest
export IMAGE_URI=gcr.io/$PROJECT_ID/$IMAGE_REPO_NAME:$IMAGE_TAG

# Do other stuff such as building source distributions
python venv/src/package setup.py sdist --dist-dir packages

# Build the image using the Dockerfile
docker build -f Dockerfile --tag $IMAGE_URI ./
```

Then source the script to create source distributions from local dependencies in a local folder named `packages` and build the image.

Behind the scenes: sourcing this script will export environment variables such as `$IMAGE_URI`. 
With this environment variable set the image is built with the command `docker build -f Dockerfile -t $IMAGE_URI ./`. 
This will send the build context to the docker engine (everything in this repository except what's inside `.dockerignore`) and 
start building the image using the instructions inside the `Dockerfile`.


To run the container's entry point and thereby start executing a main script execute the command with arguments to the dataset directory, whole-model or retraining and which pre-trained model to use:

```console
docker container run --name mycontainer --rm -it --privileged -p 6006:6006 $IMAGE_URI ARGS
```

Behind the scenes: This command will remove (`--rm`) a container previously created from the image and 
run a new one with the name `mycontainer` in interactive terminal mode `-it` and extended priviledges. 
To view web interfaces (e.g. Tensorboard during a training run) the port 6006 of the container is exposed to the host on the same port.


## Debug the container

To debug the scripts inside the container (especially the main script with different command line arguments) run one of the following:

```console
docker run -rm --name debugmycontainer -it --entrypoint "bash" $IMAGE_URI

docker run --rm --name debugmycontainer -it \
   --entrypoint "bash" \
   -e GOOGLE_APPLICATION_CREDENTIALS=/root/service-account.json \
   -v $GOOGLE_APPLICATION_CREDENTIALS:/root/service-account.json:ro \
   $IMAGE_URI
```

This will overwrite the default `ENTRYPOINT` (which would execute the main script otherwise) to execute a bash shell in a container named `debugmycontainer`.

When running the container, you will find yourself inside the `/app` directory, which is the path to the workspace of this docker container.
From there it is possible to work like you are in a linux terminal, use `cd` or execute stuff. 


## Essential Docker commands:

```console
# run commands
docker container run
docker run
```


