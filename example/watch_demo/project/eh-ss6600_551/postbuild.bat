fromelf --bin %1 --output build\bf0_ap.bin
fromelf --i32 %1 --output build\bf0_ap.hex
fromelf --text -c %1 > build\keil\Obj\bf0_ap.lst

copy build\keil\list\bf0_ap.map build /y
copy build\keil\Obj\bf0_ap.axf build /y