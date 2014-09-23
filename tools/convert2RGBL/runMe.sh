#!/bin/bash

rgbPath=/media/templeNTFS/Andrea/cloudsSeq00/RGB/PCD
gtPath=/media/templeNTFS/Andrea/cloudsSeq00/GT/PCD
outputPath=/home/gros/cls
idx0=000000
inc=000020
idxN=001980

for i in `seq -w ${idx0} ${inc} ${idxN}`
do
	./build/convert2RGBL ${rgbPath}/${i}.pcd ${gtPath}/${i}.pcd ${outputPath}/${i}.pcd
done
