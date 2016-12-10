Data = csvread('dUImage_M_Targ_Pts.csv',1);
time = Data(:,1);
Euc = Data(:,2);
dU1 = Data(:,3);
dV1 = Data(:,4);
dU2 = Data(:,5);
dV2 = Data(:,6);
dU3 = Data(:,7);
dV3 = Data(:,8);

figure
subplot(3,1,1:2);
plot(time, [dU1, dV1, dU2, dV2, dU3, dV3], 'LineWidth', 1.5);
xlabel('Time[s]');
ylabel('Pixel error');
title('Errors in U and V image coordinates');
legend('dU1','dV1','dU2','dV2','dU3','dV3')
subplot(3,1,3);
plot(time, Euc, 'LineWidth', 1.5);
xlabel('Time[s]');
ylabel('Sum of Euc. Dists.');
title('Sum of Euclidean Distances of Errors');

saveas(gcf,'MPt_following_error_vs_time','epsc');
close all;
%%
Data = csvread('dUImage_1_Targ_Pt.csv',1);
time = Data(:,1);
Euc = Data(:,2);
dU1 = Data(:,3);
dV1 = Data(:,4);

figure
subplot(3,1,1:2);
plot(time, [dU1, dV1], 'LineWidth', 1.5);
xlabel('Time[s]');
ylabel('Pixel error');
title('Errors in U and V image coordinates');
legend('dU1','dV1')
subplot(3,1,3);
plot(time, Euc, 'LineWidth', 1.5);
xlabel('Time[s]');
ylabel('Euc. Dists.');
title('Euclidean Distance of Error');

saveas(gcf,'1Pt_following_error_vs_time','epsc');