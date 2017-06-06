%% preprocess map data

clear all
close all

load('matlab_imported.mat')

avg_vals = zeros(73,4);
counts = zeros(75,1);
for i = 1:size(posx,1)
    avg_vals(posx(i),1) = avg_vals(posx(i),1) + mx(i);
    avg_vals(posx(i),2) = avg_vals(posx(i),2) + my(i);
    avg_vals(posx(i),3) = avg_vals(posx(i),3) + mz(i);
    avg_vals(posx(i),4) = avg_vals(posx(i),4) + mag(i);
    counts(posx(i)) = counts(posx(i)) + 1;
end

for i = 1:size(mag,1)
    avg_vals(i,:) = avg_vals(i,:) ./ counts(i);
end

