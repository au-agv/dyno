#!/bin/bash

python3 case.py \
    --mode ros \
    --asynchronous 1 \
    --driver autonomousNavigation \
    --launch-file dwa_validation \
    --parameters-file overrides.yaml