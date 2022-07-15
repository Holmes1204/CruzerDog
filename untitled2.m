[t1,t2] = meshgrid(-pi/4:0.01:pi/2,-2.6:0.01:-0.85);
L1 = 0.21;
L2 = 0.2;
x = -L1.*sin(t1)-L2.*sin(t1+t2);
z = -L1.*cos(t1)-L2*cos(t1+t2);
plot(x,z,'.k')
grid on