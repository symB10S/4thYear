%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Setup Camera Transformation %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Blender - 37.6651mm
f_x = 2008.8053  ;
f_y = 2008.8053 ;
p_x = 960.0000;
p_y = 540.0000 ;
s = 0;
H = 2.5;

%%% Blender Hill -25 mm
% f_x = 1333.3334;
% f_y = 1125.0000;
% p_x = 960.0000;
% p_y = 540.0000;
% s = 0;
% H = 2.5;

%%% Blender Hill - 35 mm
% f_x = 1866.6666;
% f_y = 1866.6666;
% p_x = 960.0000;
% p_y = 540.0000 ;
% s = 0;
% 
% H = 2.5;

%%% Camera Settings -35mm
% f_x = 3022.58829918849  ;
% f_y = 3023.43980585740 ;
% p_x = 972.352942326151;
% p_y = 527.209473101826 ;
% s = 0;
% H = 1.3;

%%% Camera Settings -55mm
% f_x = 4609.85361371183 ;
% f_y = 4611.98648721446 ;
% p_x = 965.405810568760 ;
% p_y = 498.545149323953 ;
% s   = 0;
% H   = 1.3;

% image_width = 1280;
% image_height = 720;

image_width  = 1920;
image_height = 1080;
image_height_buffer = 100;

maximum_trackable_objects = 10; % Maximum trackable objects
max_label_distance = 5; % Maximum distance threshold

K     = [f_x s p_x;0 f_y p_y;0 0 1];
K_inv = [f_y 0 (-p_x*f_y);0 f_x (-p_y*f_x);0 0 (f_x*f_y)]*1/((f_x).*(f_y));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  PIXEL TO DISTANCE LOOK UP TABLE   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

image_to_distance_x= [];
image_to_distance_y= [];
image_to_distance_z= [];


alpha  = deg2rad(90);
P_hill =   [0;-H;70];

p_hill = K*P_hill;
p_hill = floor(p_hill./p_hill(3));

for i = 1:p_hill(2)
    for j = 1:image_width
        
        p_in = [j;i;1];
        p_plane = [0;-H;0];
        
        normal = [0;1;0];
        
        line_vector = K_inv*p_in;
        parameter = dot(p_plane,normal)/(dot(line_vector,normal));
        p_out = line_vector*parameter;
        
        image_to_distance_x(i,j) = p_out(1) ;
        image_to_distance_y(i,j) = p_out(2) ;
        image_to_distance_z(i,j) = p_out(3) ;
        
        %image(round(p_out(1)),round(p_out(3)),:) = image_flipped(i,j,:);
    end
end
for i = p_hill(2):image_height
    for j = 1:image_width
        
        p_in = [j;i;1];
        p_plane = P_hill;
        
%         normal = [0;1;-1]; %45 Degrees
        normal = [0;cos(alpha);-sin(alpha)];
        
        line_vector = K_inv*p_in;
        parameter = dot(p_plane,normal)/(dot(line_vector,normal));
        p_out = line_vector*parameter;
        
        image_to_distance_x(i,j) = p_out(1) ;
        image_to_distance_y(i,j) = p_out(2) ;
        image_to_distance_z(i,j) = p_out(3) ;
       
        %image(round(p_out(1)),round(p_out(3)),:) = image_flipped(i,j,:);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Setup Video Reader/Writer  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

frame_scale_factor    = 0.5;
kernel_connected_size = 8;
kernel_imerode_size   = 3;
kernel_imclose_size   = 10;

kernel_imerode        = strel('disk', kernel_imerode_size);
kernel_imclose        = strel('disk', kernel_imclose_size);

minimum_object_width = 1 ; % in meters
background_threshold = 30;

% video_path = "OneVehicle/Rendered Animation/onevehiclerender.mp4";
% video_path = "OneVehicle/Rendered Animation/lane_switching.mp4";
video_path = "OneVehicle/Rendered Animation/two_lanes.mp4";
% video_path = "OneVehicle/Rendered Animation/walking_lane_switching.mp4";
% video_path = "OneVehicle/Rendered Animation/lane_switching_single.mp4";

% video_path = "OneVehicle/Rendered Animation/Braking-1.m4v";
% video_path = "OneVehicle/Rendered Animation/short_Braking-1.mp4";

% video_path = "OneVehicle/Rendered Animation/Site4_Normal.mp4";
% video_path = "OneVehicle/Rendered Animation/Site4_SwitchingLanes.m4v";

% video_path = "OneVehicle/Rendered Animation/Site3_Normal.m4v";
% video_path = "OneVehicle/Rendered Animation/Site3_Normal_Short.mp4";
% video_path = "OneVehicle/Rendered Animation/Site3_Switchinglanes.m4v";

% video_path = "4 Sites Footage/Site1/Normal/Site1_55mm_Normal.mp4";

% video_path = "4 Sites Footage/Site2/35mm_Normal/Site2_35mm_Normal.mp4";
% video_path = "4 Sites Footage/Site2/55mm_Normal/Site2_55mm_Normal.mp4";

% video_path = "OneVehicle/Rendered Animation/Dodge.mp4";
% video_path = "OneVehicle/Rendered Animation/Bugatti.mp4";

% video_path = "OneVehicle/Rendered Animation/45deg_25mm_Normal.mp4";
% video_path = "OneVehicle/Rendered Animation/30deg_35mm_Normal.mp4";
% video_path = "OneVehicle/Rendered Animation/15deg_35mm_Normal.mp4";
% video_path = "OneVehicle/Rendered Animation/15deg_35mm_Normal_OldCar.mp4";

video_input = VideoReader(video_path);

%video_writer = VideoWriter('Output/original_0p1.mp4','MPEG-4'); % Mac
video_writer = VideoWriter('Output/Site2_Normal_55_Ours');      % Linux

open(video_writer);

% background = readFrame(Vid);
background              = imread("OneVehicle/Background Image/0235.png");
% background              = imread("OneVehicle/Background Image/walking.png");

% background              = imread("OneVehicle/Background Image/Background.png");

% background              = imread("OneVehicle/Background Image/Site4_Normal_Background.png");
% background              = imread("OneVehicle/Background Image/Site4_SwitchingLanes_Background.png");

% background              = imread("OneVehicle/Background Image/Site3_Normal_Background.png");
% background              = imread("OneVehicle/Background Image/Site3_Switchinglanes_Background.png");

% background              = imread("4 Sites Footage/Site1/Normal/Site1_55mm_Normal_Background.png");

% background              = imread("4 Sites Footage/Site2/35mm_Normal/Site2_35mm_Normal_Background.png");
% background              = imread("4 Sites Footage/Site2/55mm_Normal/Site2_55mm_Normal_Background.png");

% background              = imread("OneVehicle/Background Image/Dodge_Background.png");
% background              = imread("OneVehicle/Background Image/racecar_background.png");

% background              = imread("OneVehicle/Background Image/45deg_25mm_Normal_Background.png");
% background              = imread("OneVehicle/Background Image/30deg_35mm_Normal_Background.png");
% background              = imread("OneVehicle/Background Image/15deg_35mm_Normal_Background.png");
% background              = imread("OneVehicle/Background Image/15deg_35mm_Normal_OldCar_Background.png");

background_resized      = imresize(background,frame_scale_factor);
background_resized_gray = double(rgb2gray(background_resized));

img = imshow(background_resized);
roi = images.roi.AssistedFreehand(img);
draw(roi)

mask = createMask(roi);

figure(1)
imshow(background);
figure(2)
imshow(background_resized);
figure(3)
imshow(background_resized_gray,[]);
figure(4)
imshow(mask);

%%%%%%%%%%%%%%%%%%%%%%%%%
% Define runtime Arrays %
%%%%%%%%%%%%%%%%%%%%%%%%%

time = [];

location_x = [];
location_y = [];
location_z = [];

location_x_2 = [];
location_y_2 = [];

Objects_previous = zeros( 6, maximum_trackable_objects );
Objects_current  = zeros( 6, maximum_trackable_objects );
Distance_matrix  = zeros( maximum_trackable_objects , maximum_trackable_objects );

%%%%%%%%%%%%%%%%%%%%%%
%  MAIN VIDEO LOOP   %
%%%%%%%%%%%%%%%%%%%%%%

counter   = 1;
max_label = 0;

while hasFrame(video_input)
    
    tic;
    
    %%% READ FRAME %%%
    frame_current = readFrame( video_input ); 
    frame_resized = imresize( frame_current,frame_scale_factor );
    
    %%% BACKGROUND SUBTRACTION %%%
    frame_subtracted = abs( double( rgb2gray( frame_resized ))- background_resized_gray );
    
    if counter == 80
       figure(5)
       imshow(frame_current);
       figure(6)
       imshow(frame_resized); 
       figure(7)
       imshow(frame_subtracted,[]);
    end
    
    %%% THRESHOLD IMAGE %%%
    
    % (1)----> Custom Method
    frame_subtracted( frame_subtracted <  background_threshold ) = 0;
    frame_subtracted( frame_subtracted >= background_threshold ) = 1;
    
    % (2)----> MATLAB Binarization
    %frame_subtracted = imbinarize(frame_subtracted,'adaptive','Sensitivity',0.2);
    
    if counter == 80
        figure(8)
        imshow(frame_subtracted,[]);
    end
    
    % MASK IMAGE
    frame_subtracted(~mask) = 0;
    
    if counter == 80
        figure(9)
        imshow(frame_subtracted,[]);
    end
    
    frame_subtracted = imclose(frame_subtracted,kernel_imclose); % close image --> imfill(Diff,'holes')
    frame_subtracted = imfill(frame_subtracted,'holes');
    frame_subtracted = imerode(frame_subtracted,kernel_imerode); % erode small pixels
    
%     frame_subtracted = bwareaopen(frame_subtracted, 20); % remove objects with pixels less than n
    
    cc = bwconncomp( frame_subtracted,8 ); % find connected pixels
    
    if counter == 80
        figure(10)
        imshow(frame_subtracted);
    end
    
    % Save data from previous frame
    old_max_label    = max_label;
    Objects_previous = Objects_current;  
    Distance_matrix  = NaN(maximum_trackable_objects,maximum_trackable_objects);
    
    % Bounding box
    bounding_box = regionprops(cc,'BoundingBox');
    LM = labelmatrix(cc);
    max_label = max(LM(:)); % Find max label
    
    number_removed_objects = 0 ; 
    for label_index = 1:max_label
        
        label_current = label_index - number_removed_objects; % Adjust for removed objects
        
        % bounding box --> General
        box_x1 = bounding_box(label_current).BoundingBox(1);
        box_y1 = bounding_box(label_current).BoundingBox(2);
        box_x2 = box_x1 + bounding_box(label_current).BoundingBox(3);
        box_y2 = box_y1 + bounding_box(label_current).BoundingBox(4);
        
        % co-ordinates adjusted for scale factor
%         x1 = 1 + floor(box_x1/frame_scale_factor);  
%         x2 = 1 + floor(box_x2/frame_scale_factor);
%         y  = 1 + image_height -  floor( max( box_y1 ,box_y2 )/frame_scale_factor);
        
        x1 = floor(box_x1/frame_scale_factor) - kernel_imclose_size/2;  
        x2 = floor(box_x2/frame_scale_factor) - kernel_imclose_size/2;
        y  = image_height -  floor( max( box_y1 ,box_y2 )/frame_scale_factor) - kernel_imclose_size/2;
        
        % avoid above horizon 
        if(x1<1)                              x1 = 1; end
        if(x2<1)                              x2 = 1; end
        if(x1>image_width)                    x1 = image_width; end
        if(x2>image_width)                    x2 = image_width; end
        if(y>image_height)                    y  = image_height; end
        if(y<1)                               y  = 1;            end
        
        X_1   = image_to_distance_x(y,x1);
        X_2   = image_to_distance_x(y,x2);
        Y     = image_to_distance_y(y,floor((x1+x2)/2));
        Z     = image_to_distance_z(y,floor((x1+x2)/2));
        Width = abs(X_1 - X_2);
        
        if Width > minimum_object_width % Remove object if its width is small
            
            Objects_current(1,label_current) = (X_1 + X_2)/2;  % Center x value
            Objects_current(2,label_current) = Z;              % Z distance away
            Objects_current(3,label_current) = Width;          % Width of car
            Objects_current(6,label_current) = Y;          % Width of car
            
            for old_object = 1: old_max_label
                
                Distance_matrix(label_current, old_object) = (Objects_current(1,label_current)-Objects_previous(1,old_object))^2 + (Objects_current(2,label_current)-Objects_previous(2,old_object))^2;
            
            end
            
            label = sprintf('X=%0.2f, Y=%0.2f', Objects_current(1,label_current), Objects_current(2,label_current));

            frame_resized = insertObjectAnnotation(frame_resized, ...
                            'rectangle', bounding_box(label_current).BoundingBox,...
                            label, 'Color','g');          
        else
            number_removed_objects = number_removed_objects + 1;
        end
    end
    
    if counter == 80
        figure(11)
        imshow(frame_resized);
    end
    
    max_label = max_label - number_removed_objects; % Adjust for removed objects
    
    Objects_current(:,max_label+1:maximum_trackable_objects) = 0; % Clear Previous Labelled
    list_order = 1:max_label;
    
    %%% Record Location Matrix + Persistent Labelling
    
    found_label_1 = false ; 
    found_label_2 = false ;
    
    if ( max_label > maximum_trackable_objects) 
        max_label = maximum_trackable_objects; 
    end
    
    for label_current = 1:max_label
        
        [Objects_current(5,label_current),Objects_current(4,label_current)] = min(abs(Distance_matrix(label_current,:)));
        
        if Objects_current(5,label_current) > max_label_distance % If new object
            
            Objects_current(4,label_current) = 0;
            Objects_current(5,label_current) = 0;
            
        else
            
            Objects_current(5,label_current) = Distance_matrix( label_current ,Objects_current(4, label_current ));
            
            if Objects_current(4,label_current) == 1
                
                location_x = [location_x Objects_current(1,label_current)];
                location_y = [location_y Objects_current(6,label_current)];
                location_z = [location_z Objects_current(2,label_current)];
                
                found_label_1 = true; 
                
            elseif Objects_current(4,label_current) == 2
                
                location_x_2 = [location_x_2 Objects_current(1,label_current)];
                location_y_2 = [location_y_2 Objects_current(2,label_current)];
                
                found_label_2 = true ; 
                
            end
        end
    end
    
    if ~found_label_1
        
        location_x = [location_x NaN];
        location_y = [location_y NaN];
        location_z = [location_z NaN];
        
    end
    
    if ~found_label_2
        
        location_x_2 = [location_x_2 NaN];
        location_y_2 = [location_y_2 NaN];
        
    end
    
    time = [time toc];
    
    writeVideo(video_writer,frame_resized);
%     writeVideo(video_writer,mat2gray(frame_subtracted));
    
    counter = counter + 1
end

location_total = [transpose(location_x) transpose(location_y) transpose(location_z)];
%location_total = transpose([location_x ;location_y;location_x_2 ;location_y_2]);

writematrix(location_total,'Output/output_location_15.csv');

figure(12)
plot(location_x,location_y)
xlim([-10 10])

figure(13)
plot(location_x_2,location_y_2)
xlim([-10 10])

figure(14)
scatter(location_x,location_y,'r')
xlim([-10 10])
hold on
scatter(location_x_2,location_y_2,'b')
hold off

figure(15)
scatter3(location_x,location_z,location_y);
xlim([-10 10])

close(video_writer)