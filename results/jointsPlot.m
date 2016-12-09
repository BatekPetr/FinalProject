close all
j1 = csvread('joints_1_Targ_Pt.csv',1);
jM = csvread('joints_M_Targ_Pts.csv',1);
v1 = csvread('joints_velocities_1_Targ_Pt.csv',1);
vM = csvread('joints_velocities_M_Targ_Pts.csv',1);
jlim = [3.14159, 1.64061, 3.14159, 2.49582, 4.71239, 2.0944, 6.28319];
vlim = [1.5708, 1.64061, 3.14159, 2.49582, 4.71239, 2.0944, 6.28319];

for n=2:8
    figure(1)
    plot(j1(:,1), j1(:,n)/jlim(n-1))
    hold on
    figure(2)
    plot(jM(:,1), jM(:,n)/jlim(n-1))
    hold on
    figure(3)
    plot(v1(:,1), v1(:,n)/vlim(n-1))
    hold on
    figure(4)
    plot(vM(:,1), vM(:,n)/vlim(n-1))
    hold on
end