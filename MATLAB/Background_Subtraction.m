Vid = VideoReader("Sample_Video/test_only_orange.mov");
scale_factor = 0.1;
background_threshold = 80;

vid_writer = VideoWriter('Output/test.mp4','MPEG-4');
open(vid_writer);

background = readFrame(Vid);
half_back = imresize(background,scale_factor);
half_back_gray = double(rgb2gray(half_back));

% Step through Video Frames
counter = 1;

while hasFrame(Vid)
    frame = readFrame(Vid); % Read Frame
    half = imresize(frame,scale_factor); % Half Frame Size
    
    Diff = abs(double(rgb2gray(half))-half_back_gray);
    Diff(Diff < background_threshold) = 0;
    Diff(Diff >= background_threshold) = 1;
    
    writeVideo(vid_writer,Diff);
    counter = counter + 1;
end

close(vid_writer)
