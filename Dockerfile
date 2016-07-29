FROM ubuntu:14.04
# prereqs
RUN apt-get update && apt-get install --yes git build-essential software-properties-common
RUN add-apt-repository ppa:george-edison55/cmake-3.x -y
RUN apt-get update && apt-get install --yes gawk zip python-dev python-pip python-empy cmake lib32z1 lib32ncurses5 lib32bz2-1.0 genromfs wget
RUN pip install catkin_pkg

# GCC toolchain
WORKDIR /home
RUN wget https://launchpad.net/gcc-arm-embedded/4.7/4.7-2014-q2-update/+download/gcc-arm-none-eabi-4_7-2014q2-20140408-linux.tar.bz2
RUN tar xjf gcc-arm-none-eabi-4_7-2014q2-20140408-linux.tar.bz2
ENV PATH /usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/home/gcc-arm-none-eabi-4_7-2014q2/bin/

# Install AWS cli
WORKDIR /home
RUN wget https://s3.amazonaws.com/aws-cli/awscli-bundle.zip
RUN unzip awscli-bundle.zip
RUN ./awscli-bundle/install -i /usr/local/aws -b /usr/local/bin/aws

ENV AWS_ACCESS_KEY_ID AKIAIXZSSSC5UZIXN4DA
ENV AWS_SECRET_ACCESS_KEY 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

# Get our code
ADD . /home/dev/ardupilot
WORKDIR /home/dev/ardupilot/ArduCopter
RUN make px4-v2
RUN make sitl

# Upload to S3
WORKDIR /home/dev/ardupilot/ArduCopter
RUN git rev-parse --abbrev-ref HEAD | xargs -I {} aws s3 cp ./ArduCopter-v2.px4 s3://heleport-dev/ardupilot/{}/ubuntu/
RUN git rev-parse --abbrev-ref HEAD | xargs -I {} aws s3 cp ./ArduCopter.elf s3://heleport-dev/ardupilot/{}/ubuntu/

# Copy to our output folder
CMD cp ./ArduCopter-v2.px4 /home/dev/build && cp ./ArduCopter.elf /home/dev/build