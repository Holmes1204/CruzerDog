#!/usr/bin/python3
def f2u(x,x_min,x_max,bits):
    span = x_max -x_min
    offset = x_min
    return (int) ((x-offset)*((float)((1<<bits)-1))/span)
    
def u2f(x,x_min,x_max,bits):
    span = x_max -x_min
    offset = x_min
    return (float) (((x*span)/(float)((1<<bits)-1))+offset)

# y = u2f(32767,-95.5,95.5,16)
y = f2u(-3.141592676,-95.5,95.5,16)
print(y)
# v = u2f(1000,-45,45,12)
# z = f2u(-9,0,500,12)
# print(z)
# print(y,v,z)