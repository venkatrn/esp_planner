#!/bin/bash

if [ $# -ne 2 ]; then
    echo $0: usage: dot2pdf input.dot output.pdf
    exit 1
fi

input=$1
output=$2

dot -Tps $1 -o tmp_dot.ps
ps2pdf tmp_dot.ps $2
pdfcrop $2 $2
rm tmp_dot.ps
