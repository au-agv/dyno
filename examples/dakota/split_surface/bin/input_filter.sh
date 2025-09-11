#!/bin/bash

python3 case.py \
    --input-filter \
    --parameters-file ${1} \
    --output-files case.json transmission.json \
    --python-includes get_cwd.py