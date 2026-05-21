#!/usr/bin/env bash

set -e

cd "$(dirname "$0")"

rsync -av --stats --progress \
    ../build/html/ \
    robot@au-agv.github.io:/home/robot/docs/dyno/
