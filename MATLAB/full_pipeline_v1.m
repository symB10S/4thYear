f_x = 2008.8053  ;
f_y = 2008.8053 ;
p_x = 960.0000;
p_y = 540.0000 ;
s = 0;

image_width = 1920;
image_height = 1080;

K = [f_x s p_x;0 f_y p_y;0 0 1];
K_inv = [f_y 0 (-p_x*f_y);0 f_x (-p_y*f_x);0 0 (f_x*f_y)]*1/((f_x).*(f_y));

video_path = "OneVehicle/Rendered Animation/onevehiclerender.mkv";

Vid = VideoReader(video_path);
scale_factor = 0.5;
max_width = scale_factor*image_width/10;
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
    frame = readFrame(Vid); % Read Frame
    half = imresize(frame,scale_factor); % Half Frame Size
    
    Diff = abs(double(rgb2gray(half))-half_back_gray);
    Diff(Diff < background_threshold) = 0;
    Diff(Diff >= background_threshold) = 1;
    
    %Diff = bwareaopen(Diff, 50);
    Diff = imclose(Diff,se);
    
    cc = bwconncomp(Diff,4); % find connected pixels
    numPixels = cellfun(@numel,cc.PixelIdxList);
    
    %S = regionprops(cc, 'Area'); % label the area 
    %labeled = labelmatrix(cc);
    
    
    %Feret Parameters
    [out,LM] = bwferet(cc,'MaxFeretProperties'); % Get Feret Properies
    maxLabel = max(LM(:)); % Find max label
    
    Objects = zeros(3,maxLabel);
        
    for labelvalues = 1:maxLabel
        out.MaxDiameter(labelvalues);
        Diff = insertShape(Diff,'Line',[out.MaxCoordinates{labelvalues}(1,1) out.MaxCoordinates{labelvalues}(1,2) out.MaxCoordinates{labelvalues}(2,1) out.MaxCoordinates{labelvalues}(2,2)],'LineWidth',5,'Color','green');
        %Diff = insertShape(Diff,'Line',[out.MinCoordinates{labelvalues}(1,1) out.MinCoordinates{labelvalues}(1,2) out.MinCoordinates{labelvalues}(2,1) out.MinCoordinates{labelvalues}(2,2)],'LineWidth',5);
    end
    
    writeVideo(vid_writer,Diff);
    counter = counter + 1;
end

close(vid_writer)