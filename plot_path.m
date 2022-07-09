data = importdata("path.csv");
size(data);
p = data(1:2:end,:);
v = data(2:2:end,:);
subplot(2,1,1)
% plot(p)
plot(p(:,1),p(:,3))
% legend(["p_x" "p_y" "p_z"])
grid on
subplot(2,1,2)
plot(v(:,1),v(:,3))
% plot(v)
% legend(["v_x" "v_y" "v_z"])
grid on
I = [0.0158533 -3.66e-05 -6.11e-05;
    -3.66e-05 0.0377999 -2.75e-05;
    -6.11e-05 -2.75e-05 0.0456542;];