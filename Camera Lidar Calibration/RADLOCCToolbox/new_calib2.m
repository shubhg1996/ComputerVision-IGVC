%input file for RADLOCC
addpath('RADLOCCToolbox');
savepath;

laserdatafname='matlabLaserHorizontalData.txt';
videoframetimefname='VideoLogAsciiCalibration.txt';

if ~exist(laserdatafname,'file') || ~exist(videoframetimefname,'file')
    disp('File does not exist.');
    return;
end

rawlaserdata=load(laserdatafname);
videotimestamps=load(videoframetimefname);
videotimestamps=videotimestamps(:,1);
disp('LiDAR files loaded');

% get laser data closest to video timestamps
[lasertimestamps,imindices] = GetClosestTimeStamps(videotimestamps, rawlaserdata(:,1) );
laserdata = rawlaserdata(imindices,:);

% get range matrix and angle vector
[rangeMatrix, angleVector, laserDivisor] = QBuildLaserRangeAngle(laserdata);
rawrangeMatrix=QBuildLaserRangeAngle(rawlaserdata);

roughth = 0.05;
fineth = 0.02;

auto_select_cb;

if length(selectionnumbers)>14
    calibrate_cb;
    save_cb;
    flag = 1;
else    
    disp('insufficient and corrupt data, take readings again');
    flag = 0;
end

