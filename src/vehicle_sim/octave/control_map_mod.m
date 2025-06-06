clear
clc

filename = "../src/vehicle_control_map_bruteSolver/build/control_map.csv";
t = csvread(filename);
firstline = fileread(filename);
firstline = strsplit(firstline, "\n");
firstline = firstline{1};

invalids = find(abs(t(:,5)) > 1);
t(invalids, 5);
t(invalids, 3) = 1000;

outfile = strrep(filename, ".csv", "2.csv");
fid = fopen(outfile, 'w');
fprintf(fid, '%s\n', firstline);
fclose(fid);
dlmwrite(outfile, t, '-append');
