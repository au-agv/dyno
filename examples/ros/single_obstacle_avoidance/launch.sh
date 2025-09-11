#!/bin/bash

case ${1} in

  run)
    echo -n "Launching Dakota case ..."
    dakota case.in
    ;;

  clean)
    echo "Cleaning Dakota workspace ..."
    rm -rf runs
    rm -f *.h5
    rm -f *.rst
    ;;

  *)
    echo -n "Invalid verb."
    ;;
esac
