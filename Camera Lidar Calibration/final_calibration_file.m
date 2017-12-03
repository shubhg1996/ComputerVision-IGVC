new_calib;
new_calib2;

if flag==1

    fileID = fopen('final_results.txt','w');

    fprintf(fileID,'phi\n');
    [nrows,ncols] = size(phi);
    for row = 1:nrows
        fprintf(fileID, ' %d' , phi(row,:));
        fprintf(fileID,'\n');
    end

    fprintf(fileID,'phi inverse\n');
    inv_phi = inv(phi);
    [nrows,ncols] = size(inv_phi);
    for row = 1:nrows
        fprintf(fileID, ' %d' , inv_phi(row,:));
        fprintf(fileID,'\n');
    end

    fprintf(fileID, 'delta\n');
    for row = 1:nrows
        fprintf(fileID, ' %d' , delta(row,:));
        fprintf(fileID,'\n');
    end
    fclose(fileID);
end