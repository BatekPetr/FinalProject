clear all, close all, clc;

%% Single Point Following
Data = csvread('MarkerMotionMedium_img_error_1_Targ_Pt.csv',1);
deltaTs = Data(:,1);
maxEuc_dUpixels = Data(:,2);
max_dUpixels = Data(:,3);
max_dVpixels = Data(:,4);

figure
subplot(2,1,1)
plot(deltaTs,[maxEuc_dUpixels, max_dUpixels, max_dVpixels], 'LineWidth',1.5)
set ( gca, 'xdir', 'reverse' )
title('Single point tracking')
xlabel('deltaT [s]')
ylabel('max error [pixels]')
legend('Euclidean dist', 'Err in du', 'Err in dv','Location','northeastoutside')

%% Multiple points following
clear all;
Data = csvread('MarkerMotionMedium_img_error_M_Targ_Pts.csv',1);
deltaTs = Data(:,1);
maxEuc_dUpixels = Data(:,2);
max_dU_1pixels = Data(:,3);
max_dV_1pixels = Data(:,4);
max_dU_2pixels = Data(:,5);
max_dV_2pixels = Data(:,6);
max_dU_3pixels = Data(:,7);
max_dV_3pixels = Data(:,8);

subplot(2,1,2)
plot(deltaTs,[maxEuc_dUpixels, max_dU_1pixels, max_dV_1pixels, max_dU_2pixels, max_dV_2pixels, max_dU_3pixels, max_dV_3pixels], 'LineWidth',1.5)
set ( gca, 'xdir', 'reverse' )
title('Tracking of 3 points')
xlabel('deltaT [s]')
ylabel('max error [pixels]')
legend('Sum of Euc. dists', 'Err in du Pt 1', 'Err in dv Pt 1', 'Err in du Pt 2', 'Err in dv Pt 2', 'Err in du Pt 3', 'Err in dv Pt 3','Location','northeastoutside');

saveas(gcf,'errors','epsc');
%close all;