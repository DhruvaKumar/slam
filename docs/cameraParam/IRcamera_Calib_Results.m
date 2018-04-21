% Intrinsic and Extrinsic Camera Parameters
% 
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 364.457362485643273 ; 364.542810626989194 ];

%-- Principal point:
cc = [ 258.422487561914693 ; 202.487139940005989 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.098069182739161 ; -0.249308515140031 ; 0.000500420465085 ; 0.000529487524259 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 1.569282671152671 ; 1.461154863082004 ];

%-- Principal point uncertainty:
cc_error = [ 2.286222691982841 ; 1.902443125481905 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.012730833002324 ; 0.038827084194026 ; 0.001933599829770 ; 0.002380503971426 ; 0.000000000000000 ];

%-- Image size:
nx = 512;
ny = 424;
