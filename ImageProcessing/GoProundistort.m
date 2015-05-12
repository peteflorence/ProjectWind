%start = 1;
%last = 1317;

load('cameraParams_1080_120_W.mat');

%for i = start:last
i = 0;
nextframeexist = 2;

for folder = 4:26
  
  nextframeexist = 2;
  
  while nextframeexist ~= 0
    
    i = i+1;
    %vid = VideoReader('name_of_video');
    %im = read(vid,ind_frame);
    
    inpath = strcat('/Users/pflomacpro/ProjectWind/ImageProcessing/0423vortex/',num2str(folder),'/in_imagefolder/');
    imnamein = strcat(inpath,'videoframe',num2str(i),'.bmp');
    im = imread(imnamein);
    
    im_undistorted = undistortImage(im, cameraParameters);
    
    outpath = strcat('/Users/pflomacpro/ProjectWind/ImageProcessing/0423vortex/',num2str(folder),'/out_imagefolder/');
    imnameout = strcat(outpath,'UDvideoframe',num2str(i),'.bmp');
    imwrite(im_undistorted,imnameout,'BMP');
    
    
    %imshow(im)
    %figure
    %imshow(im_undistorted)
    
    
    nextframe = strcat(inpath,'videoframe',num2str(i+1),'.bmp');
    nextframeexist = exist(nextframe, 'file');
    
  end
  
end



