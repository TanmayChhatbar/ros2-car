function data_out = loadcsv(filename)
  data = csvread(filename);
  while all(data(1,:)==0)
    data = data(2:end, :);
  endwhile
  f = fopen(filename);
  firstline = fgetl(f);
  titles = strsplit(firstline, ',');
  data_out = struct();
  for i = 1:length(titles)
    tmp = titles{i};
    tmp = strrep(tmp, " ", "");
    data_out.(tmp) = data(:,i);
  endfor
endfunction
