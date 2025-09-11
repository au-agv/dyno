#!/bin/bash

python3 case.py \
    --mode ros \
    --input-filter \
    --parameters-file ${1} \
    --python-includes python_includes.py \
    --output-files overrides.yaml case.json