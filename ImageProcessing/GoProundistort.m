start = 200;
last = 450;

load('cameraParams_1080_120_W.mat');

for i = start:last
  
  
  %vid = VideoReader('name_of_video');
  %im = read(vid,ind_frame);
  
  inpath = '/Users/pflomacpro/ProjectWind/ImageProcessing/in_imagefolder/';
  imnamein = strcat(inpath,'videoframe',num2str(i),'.bmp');
  im = imread(imnamein);
  
  im_undistorted = undistortImage(im, cameraParameters);
  
  outpath = '/Users/pflomacpro/ProjectWind/ImageProcessing/out_imagefolder/';
  imnameout = strcat(outpath,'UDvideoframe',num2str(i),'.bmp');
  imwrite(im_undistorted,imnameout,'BMP');
  
  
  %imshow(im)
  %figure
  %imshow(im_undistorted)
  
end


