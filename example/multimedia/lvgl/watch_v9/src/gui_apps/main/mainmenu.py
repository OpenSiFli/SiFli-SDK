import sys

def generateR2M(rw, mShift, s):
    r_2_m = []
    rw = float(rw)
    for r in range (0, (int)(1.8*rw)):
        if r == 0:
            m = 2**12;
        else:
            #m = 1.1111*((r*120/rw) - 0.4342944819E-1*s*120*10.**((r*120/rw)/(s*120.0))+0.043429*s*120)/(r*120/rw)

            m = 1.11111*((r*120/rw)-0.4342944819E-1*s*120*10.**((r*120/rw)/(s*120.0))+0.043429*s*120)/(r*120/rw)
            m = (int)(m * (2**mShift))
            r_2_m.append(m)
    print 'r_2_m[',(int)(1.8*rw),']:\n', r_2_m

def generateR2D(rw, nrShift, s):
    r_2_d = []
    rw = float(rw)
    for r in range (0, (int)(1.8*rw)):
        dp = (7.0/22 - (7.0/220) * 10**(r/(rw*s)))/(63.0/220)
        dp = (int)(dp * (2**nrShift))
        r_2_d.append(dp)
    print 'r_2_d[',(int)(1.8*rw),']:\n', r_2_d

if __name__ == '__main__':
    s = 1.8;
    rw, shift= (int)(sys.argv[1]), (int)(sys.argv[2])
    generateR2M(rw, shift, s)
    generateR2D(rw, shift, s)
    


