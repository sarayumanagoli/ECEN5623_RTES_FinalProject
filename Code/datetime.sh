#!/bin/sh
FILEDATE=`date`
sudo convert $4 -font /usr/share/fonts/truetype/noto/NotoSans-Regular.ttf -pointsize 24 -fill white -undercolor '#00000080' -gravity SouthEast -annotate +0+5 "$FILEDATE\n$1\n$2\n$3" $4
