#! /bin/bash
set -e
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
mkdir "$DIR/../build"
OUTPUT_PATH=$( cd "$DIR/../build" && pwd )
docker run -a stderr -a stdout -v $OUTPUT_PATH:/home/dev/build ardupilot-build