if exist %1\ER_IROM1.hex (
    echo r > %1\download.jlink
    rem copy %1\ER_IROM1 %1\ER_IROM1.hex; 
    echo loadfile %1\ER_IROM1.hex >> %1\download.jlink
    if exist %1\ER_IROM2.hex (
        rem copy %1\ER_IROM2 %1\ER_IROM2.hex; 
        echo loadfile %1\ER_IROM2.hex >> %1\download.jlink
    )
    if exist %1\ER_IROM3.hex (
        rem copy %1\ER_IROM3 %1\ER_IROM3.hex; 
        echo loadfile %1\ER_IROM3.hex >> %1\download.jlink
    )
    echo exit >> %1\download.jlink 
    jlink.exe -device SF32LB55X -if SWD -speed 10000 -autoconnect 1 -CommandFile %1\download.jlink
)else (
    copy %1 %~dpn1.hex
    echo r > download.jlink
    echo loadfile %~dpn1.hex >> download.jlink
    echo exit >> download.jlink 
    jlink.exe -device SF32LB55X -if SWD -speed 10000 -autoconnect 1 -CommandFile download.jlink
)
