%start = 1;
%last = 1317;

load Hero3Calibration/cameraParamsHero3_720_120fps.mat

%for i = start:last
i = 0;
nextframeexist = 2;

for folder = 5:5
  
  i = 0;
  nextframeexist = 2;
  
  while nextframeexist ~= 0
    
    i = i+1;
    %vid = VideoReader('name_of_video');
    %im = read(vid,ind_frame);
    
    %inpath = strcat('/Users/pflomacpro/ProjectWind/ImageProcessing/0423vortex/',num2str(folder),'/in_imagefolder/');
    inpath = strcat('/Users/pflomacpro/RLG/Data/15-09-01/Strength4/videofolder/');
    imnamein = strcat(inpath,'videoframe',num2str(i),'.bmp');
    im = imread(imnamein);
    
    im_undistorted = undistortImage(im, cameraParamsHero3_720_120fps);
    
    %outpath = strcat('/Users/pflomacpro/ProjectWind/ImageProcessing/0423vortex/',num2str(folder),'/out_imagefolder/');
    outpath = strcat('/Users/pflomacpro/RLG/Data/15-09-01/Strength4/out_imagefolder/');
    imnameout = strcat(outpath,'UDvideoframe',num2str(i),'.bmp');
    imwrite(im_undistorted,imnameout,'BMP');
    
    
    %imshow(im)
    %figure
    %imshow(im_undistorted)
    
    
    nextframe = strcat(inpath,'videoframe',num2str(i+1),'.bmp');
    nextframeexist = exist(nextframe, 'file');
    
  end
  
end



