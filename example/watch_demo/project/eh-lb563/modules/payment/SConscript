from building import *

src = Glob('*.c') + Glob('*.cpp')
cwd = GetCurrentDir()

CPPPATH = [cwd]
group = DefineGroup('', src, depend = [''], CPPPATH=CPPPATH)

Return('group')
