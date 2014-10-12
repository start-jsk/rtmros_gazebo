#!/bin/bash

function error {
    exit 1
}
trap error ERR

OUTPUT_FILE=$1
