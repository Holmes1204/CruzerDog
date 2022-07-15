data = importdata("path.csv");
size(data);
% p = data(1:2:end,:);
% v = data(2:2:end,:);
% plot(p)
% hold on
% plot(v)
% subplot(2,1,1)
% % plot(p)
% plot(p(:,1),p(:,3))
% % legend(["p_x" "p_y" "p_z"])
% grid on
% subplot(2,1,2)
% plot(v(:,1),v(:,3))
% % plot(v)
% % legend(["v_x" "v_y" "v_z"])
% grid on
% I = [0.0158533 -3.66e-05 -6.11e-05;
%     -3.66e-05 0.0377999 -2.75e-05;
%     -6.11e-05 -2.75e-05 0.0456542;];
% des print
p_des = data(1:2:end,:);
p = data(2:2:end,:);
l = min(size(p_des,1),size(p,1));
subplot(2,1,1)
plot(2:l,p_des(2:l,1));
hold on
plot(2:l,p(2:l,1));
grid on
legend(["pdes_x","p_x"])

subplot(2,1,2)
plot(2:l,p_des(2:l,3));
hold on
plot(2:l,p(2:l,3));
grid on
legend(["pdes_z","p_z"])




