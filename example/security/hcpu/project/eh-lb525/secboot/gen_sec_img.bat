@set WORK_PATH=%~dp0
@set CURR_PATH=%cd%
@cd %WORK_PATH%
python imgtool.py enc_image --key=sifli01/s01 --uid=sifli01/uid --img=../build/bf0_ap.bin --eimg=image_sec --sigkey=sifli01/sig
python imgtool.py dec_image --key=sifli01/s01 --uid=sifli01/uid --img=../build/bf0_ap.bin --eimg=image_sec --sigkey=sifli01/sig
python imgtool.py gen_ftab --table=../build/ftab/ftab.bin --eimg=image_sec --sigkey=sifli01/sig
@cd %CURR_PATH%