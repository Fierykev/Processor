#!/bin/bash

/u/gheith/public/ppc64-linux/bin/as $USER.S -o $USER.o
/u/gheith/public/ppc64-linux/bin/objcopy -O binary --only-section=.text $USER.o /dev/stdout | xxd -p - | tr -d '\n' | sed -e "s/.\{16\}/&\n/g" | perl -lpe '$_ .= "0" x (16 - length $_)' | tee $USER.bin
