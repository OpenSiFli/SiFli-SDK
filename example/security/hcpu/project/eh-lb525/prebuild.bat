call ..\..\..\..\set_env.bat
call scons resource --no_cc -j8
scons build/ftab -j8