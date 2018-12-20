clear;

rows = 480; cols = 640;

x = floor(cols/(14.5)); y = floor(rows*(0.6));

fid = fopen('keyboard_data.mif','w');

fprintf(fid,'-- %3ux%3u 8bit image color values\n\n',rows,cols);
fprintf(fid,'WIDTH = 8;\n');
fprintf(fid,'DEPTH = %4u;\n\n',rows*cols);
fprintf(fid,'ADDRESS_RADIX = HEX;\n');
fprintf(fid,'DATA_RADIX = HEX;\n\n');
fprintf(fid,'CONTENT BEGIN\n');

count = 0;
for r = 1:rows
    for c = 1:cols
        if (((r <= y) && (((c <= 2.5*x) && (c >= x)) || ((c <= 5*x) && (c >= 3.5*x))...
            || ((c <= 8.5*x) && (c >= 7*x)) || ((c <= 11*x) && (c >= 9.5*x)) ||...
            ((c <= 13.5*x) && (c >= 12*x))))...
            || ((((c == 2*x) || (c == 4*x) || (c == 8*x) || (c == 10*x) || (c == 12.5*x)) && (r >= y))...
                || (c == 6*x)))
            fprintf(fid,'%x : 00;\n',count);
        else 
            fprintf(fid,'%x : 01;\n',count);
        end
        count = count + 1;
    end
end
fprintf(fid,'END;');
fclose(fid);