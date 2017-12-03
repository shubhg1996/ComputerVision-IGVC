%input file
addpath('RADOCCToolbox');
addpath('RADOCCToolbox/CornerFinder');
savepath;

clear fc cc kc KK
calib_name = 'image';
format_image = 'bmp';
check_directory;

if (Nima_valid~=0),
    % Reading images:
    ima_read_calib;
    if ~isempty(ind_read),
       mosaic;
    end;
end;

ima_read_calib;

dX = 35;
dY = 35;

click_calib;

%grid corners extracted

go_calib_optim;

saving_calib;

export_calib_data;