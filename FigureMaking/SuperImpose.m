


for i = 1:2
  
  %vid = VideoReader('name_of_video');
  %im = read(vid,ind_frame);
  
  inpath = strcat('/Users/pflomacpro/ProjectWind/FigureMaking/Ben1/');
  imnamein = strcat(inpath,num2str(i),'.png');
  im{i} = imread(imnamein);
  
  
  
  %outpath = strcat('/Users/pflomacpro/ProjectWind/FigureMaking/Ben1/');
  %imnameout = strcat(inpath,num2str(i),'super.png');
  %imwrite(im_undistorted,imnameout,'PNG');
  
  
  
  
  
  
end




imshow(im{i})
figure
%imshow(im_undistorted)