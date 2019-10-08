# Definition of Submission container

# We start from a base ROS image
FROM duckietown/dt-core:daffy-amd64

# DO NOT MODIFY: your submission won't run if you do
RUN apt-get update -y && apt-get install -y --no-install-recommends \
         gcc \
         libc-dev\
         git \
         bzip2 \
         python-tk \
         python-wheel \
         python-pip && \
     rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# here, we install the requirements, some requirements come by default
# you can add more if you need to in requirements.txt
COPY requirements.txt .
RUN pip install -r requirements.txt

# For ROS Agent - Need to upgrade Pillow for Old ROS stack
RUN pip install pillow --user --upgrade

# let's copy all our solution files to our workspace
# if you have more file use the COPY command to move them to the workspace
COPY solution.py ./

# For ROS Agent - Additional Files
COPY rosagent.py ./
COPY template.launch ./

RUN /bin/bash -c "export PYTHONPATH="/usr/local/lib/python2.7/dist-packages:$PYTHONPATH""

# For ROS Agent - pulls the default configuration files 
# Think of this as the vehicle name
ENV HOSTNAME=default

# let's see what you've got there...

ENV DISABLE_CONTRACTS=1
CMD ["python", "solution.py"]