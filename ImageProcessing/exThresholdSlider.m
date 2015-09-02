function exThresholdSlider

%X = imread('/Users/pflomacpro/ProjectWind/ImageProcessing/0423vortex/1/out_imagefolder/UDvideoframe100.bmp');
X = imread('UDvideoframe1.bmp');
figure(1); imshow(X)
%Y = imread('/Users/pflomacpro/ProjectWind/ImageProcessing/0423vortex/1/out_imagefolder/UDvideoframe223.bmp');
Y = imread('UDvideoframe450.bmp');
figure(2); imshow(Y)
Z = imsubtract(Y,X);
figure(3); imshow(Z)

%gray = rgb2gray(Z);
%figure(4); imshow(gray)
%level = graythresh(Z);




level = 0.03
BW = im2bw(Z, level);
figure(5); imshow(BW)
f = figure(6)
b = uicontrol('Parent',f,'Style','slider','Position',[81,54,419,23],...
              'value',level, 'min',0, 'max',0.3);         
           
set(b,'Callback',@(es,ed) updateAndPlot(es.Value)); 


function updateAndPlot(level)
BW = im2bw(Z, level);
figure(5); imshow(BW)
end

end
