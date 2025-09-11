#!/bin/bash

case ${1} in

run)
  printf "Launching Dakota case ...\n"
  dakota case.in
  ;;

clean)
  printf "Cleaning Dakota workspace ...\n"
  rm -rf runs
  rm -f *.h5
  rm -f *.rst
  ;;

save)
  printf "Saving Dakota workspace ...\n"
  find ./runs -maxdepth 2 -type l -delete
  find ./runs/*/variables.json -delete
  tar czf saved/$(basename ${PWD})-$(date '+%Y-%m-%d-%H-%M-%S').tar.gz case.in case.h5 runs/*
  rm -rf runs
  rm -f *.h5
  rm -f *.rst
  ;;

*)
  printf "Invalid verb.\n"
  ;;
esac
