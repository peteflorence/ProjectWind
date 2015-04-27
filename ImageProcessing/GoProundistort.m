load('cameraParams_1080_120_W.mat');

%vid = VideoReader('name_of_video');
%im = read(vid,ind_frame);

im = imread('videoframe1.bmp');
im_undistorted = undistortImage(im, cameraParameters);
imwrite(im_undistorted,'videoframe1-UD','BMP');


%imshow(im)
%figure
%imshow(im_undistorted)

