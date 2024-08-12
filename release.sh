#!/bin/bash
rm -rf release
mkdir -p release

cp -rf Led *.{hpp,cpp,txt,json} LICENSE release/

mv release score-addon-led
7z a score-addon-led.zip score-addon-led
