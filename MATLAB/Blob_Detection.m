blob_width = 5;
blob_height = 5;
blobs = [];

Vid = VideoReader("Output/test.mp4");
background = double(rgb2gray(readFrame(Vid)));

vid_writer = VideoWriter('Output/test_labelled.mp4','MPEG-4');
open(vid_writer);

[x_size,y_size] =size(background);

se = strel('disk',10);

% Step through Video
counter = 1; 

while hasFrame(Vid)
    frame = double(rgb2gray(readFrame(Vid))); % Read Frame
    frame_close = imclose(frame,se);
    %s = regionprops(frame,'centroid');
    
    cc = bwconncomp(frame_close,4);
    labeled = labelmatrix(cc);
    RGB_label = label2rgb(labeled,'spring','c','shuffle');
    
    writeVideo(vid_writer,RGB_label);
    counter = counter + 1;
    
end


close(vid_writer)

imshow(RGB_label)
%centroids = cat(1,s.Centroid);

%imshow(frame)
%hold on
%plot(centroids(:,1),centroids(:,2),'b*')
%hold off