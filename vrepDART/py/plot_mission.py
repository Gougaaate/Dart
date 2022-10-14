import numpy as np
import matplotlib.pyplot as plt
import sys

# to cope with french coma instead of decimal point
def cvt_float(st):
    try:
        v = float(st)
    except:
        v = float(st.replace(',', '.'))
    return v

vx=[]
vy=[]
vtht=[]
vt=[]

vsonf=[]
tsonf=[]
vsonl=[]
tsonl=[]
vsonr=[]
tsonr=[]
vbat=[]
tbat=[]

nmax = -200
n = 0

try:
    flog = open (sys.argv[1],"r")
except:
    flog = open ("../../../logs/rob1a.log","r")
st = flog.readline()
while True:
    st = flog.readline()
    if len(st) == 0:
        break
    v = st[0:-1].split(";")
    timsim=cvt_float(v[0])
    tim=cvt_float(v[1])
    logid=v[2]
    if logid == "robot pose":
        vx.append(cvt_float(v[3]))
        vy.append(cvt_float(v[4]))
        vt.append(timsim)
        vtht.append(cvt_float(v[5]))
        n = n+1
        if n == nmax:
            break
    elif logid == "sonar front":
        tsonf.append(timsim)
        vsonf.append(cvt_float(v[3]))
    elif logid == "sonar left":
        tsonl.append(timsim)
        vsonl.append(cvt_float(v[3]))
    elif logid == "sonar right":
        tsonr.append(timsim)
        vsonr.append(cvt_float(v[3]))
    elif logid == "battery":
        tbat.append(timsim)
        vbat.append(cvt_float(v[3]))
        
flog.close()

print (len(vx)," track points")

vx = np.asarray(vx)
vy = np.asarray(vy)
vtht = np.asarray(vtht)
vhd = 90.0-vtht
ii = np.where(vhd<0.0)
vhd[ii] = vhd[ii]+360.0
ii = np.where(vhd>=360.0)
vhd[ii] = vhd[ii]-360.0

#for i in range(len(vtht)):
#    print (vtht[i],vhd[i])

vt =  np.asarray(vt)

plt.figure(1)
plt.xlim([-0.5+np.round(np.min(vx)*2.0)/2.0, 0.5+np.round(np.max(vx)*2)/2.0])
plt.ylim([-0.5+np.round(np.min(vy)*2.0)/2.0, 0.5+np.round(np.max(vy)*2)/2.0])
plt.plot(vx,vy)
plt.plot(vx,vy,'*b')
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title("robot track (duration = %ds)"%(int(round(np.max(vt)))))
plt.savefig('robot_log_track.png')
plt.figure(2)
plt.plot (vt,vhd,'*b')
plt.xlabel("t (s)")
plt.ylabel("heading (degrees)")
plt.title("robot heading")
plt.figure(3)
plt.plot (tsonf,vsonf,'*b')
plt.xlabel("t (s)")
plt.ylabel("front sonar (m)")
plt.title("front sonar")
plt.figure(4)
plt.plot (tsonl,vsonl,'*b')
plt.xlabel("t (s)")
plt.ylabel("left sonar (m)")
plt.title("left sonar")
plt.figure(5)
plt.plot (tsonr,vsonr,'*b')
plt.xlabel("t (s)")
plt.ylabel("right sonar (m)")
plt.title("right sonar")
plt.savefig('robot_log_right.png')
plt.figure(6)
plt.plot (tbat,vbat,'*b')
plt.xlabel("t (s)")
plt.ylabel("battery level (%)")
plt.title("battery level")
plt.savefig('robot_log_battery.png')

plt.show()
