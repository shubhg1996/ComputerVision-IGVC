This matlab code finds the intrinsic and extrinic calibration matrix between camera and LiDAR. 
The input is a set of images and subsequent timestamps and laser data(obtained through mentioned rosnode) 
and the output is the phi and delta matrix, relation between which is mentioned below.

Run rosnode radlocc_snapshotter to record the timestamp, laser scans, camera_images;
In separate terminals, open LiDAR and Camera.
Save images created by the rosnode in the parent folder (alongside the the two toolbox folder, final calibration file and the readme file)
Make two files named matlabLaserHorizontalData.txt and VideoLogAsciiCalibration.txt and save in the parent folder.
In matlab, home ---> set path ---> */RADOCCToolbox
                              ---> */RADOCCToolbox/CornerFinder
                              ---> */RADLOCCToolbox
                              ---> */RADLOCCToolbox/Functions
                                    where */ is the parent folder.
Run final_calibration_file;
The resulting matrix is stored in final_results.txt.
To see the results, type final_results.txt in matlab or open the file.


the delta and phi are related as follows:

A point in 3D space with coordinate vector Pc with respect to the camera has coordinate vector Pl in the laser frame. The two vectors can be related by the following equation:

Pl=Φ(Pc-Δ)
where Δ is the translation offset and Φ is the rotation matrix defined by the set of three Euler angles φx, φy and φz.

Φ defines the rotation that takes the camera frame to the laser frame. It is a combination of three rotations (the order is important). The first is a rotation about the x axis by the angle φx, the second is about the (possibly new) y axis by the angle φy and the last is that about the (again possibly new) z axis by the angle φz. Δ is simply the coordinates of the vector extending from the camera origin to the laser origin in the camera’s coordinate frame.
//courtesy: http://www-personal.acfr.usyd.edu.au/akas9185/AutoCalib/index.html