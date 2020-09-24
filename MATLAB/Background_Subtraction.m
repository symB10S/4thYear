video_path = "OneVehicle/Rendered Animation/onevehiclerender.mkv";

Vid = VideoReader(video_path);
scale_factor = 0.1;
background_threshold =  10;

vid_writer = VideoWriter('Output/original_0p1');
open(vid_writer);

%background = readFrame(Vid);
background = imread("OneVehicle/Background Image/0235.png");
half_back = imresize(background,scale_factor);
half_back_gray = double(rgb2gray(half_back));

se = strel('disk',10);

% Step through Video Frames
counter = 1;

while hasFrame(Vid)
    tic
    frame = readFrame(Vid); % Read Frame
    half = imresize(frame,scale_factor); % Half Frame Size
    
    Diff = abs(double(rgb2gray(half))-half_back_gray);
    Diff(Diff < background_threshold) = 0;
    Diff(Diff >= background_threshold) = 1;
    
    %Diff = imclose(Diff,se);
    cc = bwconncomp(Diff,4);
    labeled = labelmatrix(cc);
    toc
    
    
    writeVideo(vid_writer,Diff);
    counter = counter + 1;
end

close(vid_writer)