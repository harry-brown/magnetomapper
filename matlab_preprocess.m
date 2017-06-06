%% preprocess map data

clear all
close all

load('matlab_imported.mat')

avg_vals = zeros(73,4);
counts = zeros(73,1);
for i = 1:size(posx,1)
    avg_vals(posx(i),1) = avg_vals(posx(i),1) + mx(i);
    avg_vals(posx(i),2) = avg_vals(posx(i),2) + my(i);
    avg_vals(posx(i),3) = avg_vals(posx(i),3) + mz(i);
    avg_vals(posx(i),4) = avg_vals(posx(i),4) + mag(i);
    counts(posx(i)) = counts(posx(i)) + 1;
end

for i = 1:size(avg_vals,1)
    avg_vals(i,:) = avg_vals(i,:) ./ counts(i);
end

avg_vals = round(avg_vals/10) * 10;
x = 1:73;
figure(1);
hold on
plot(x,avg_vals(:,1),'LineWidth',2);
plot(x,avg_vals(:,2),'LineWidth',2);
plot(x,avg_vals(:,3),'LineWidth',2);

csvwrite('1d_map.csv',avg_vals);