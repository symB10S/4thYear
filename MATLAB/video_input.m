clear cam

cam = webcam(1);
counter = 0;
vidWriter = VideoWriter('frames.avi');
open(vidWriter);

fig = figure('NumberTitle','off','MenuBar','none');
fig.Name = 'My Camera';
ax = axes(fig); 
axis(ax,'image');
img = image(ax,zeros(size(frame),'uint8')); 
%preview(cam,img)

background_bw = rgb2gray(background);

while counter <= 100
    % Acquire frame for processing
    frame = snapshot(cam);
    % Write frame to video
    writeVideo(vidWriter,frame);
    
    % Subtract the images
    Diff = abs(double(rgb2gray(frame))-double(background_bw));
    
    Diff(Diff < 50) = 0;
    
    
    imshow(Diff);
    
    counter = counter + 1;
end

close(vidWriter);
clear cam