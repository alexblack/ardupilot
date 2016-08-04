#! /bin/bash
set -e
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
cd $DIR
cd ..

if [ -z $AWS_ACCESS_KEY_ID ]; then
  echo 'Missing env setting AWS_ACCESS_KEY_ID'
  exit 1
fi

if [ -z $AWS_SECRET_ACCESS_KEY ]; then
  echo 'Missing env setting AWS_SECRET_ACCESS_KEY'
  exit 1
fi

docker build -t ardupilot-build --build-arg AWS_ACCESS_KEY_ID=$AWS_ACCESS_KEY_ID --build-arg AWS_SECRET_ACCESS_KEY=$AWS_SECRET_ACCESS_KEY .