%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     Input Parameters        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

frame_scale_factor    = 0.5;
kernel_imclose        = strel('disk',10);
kernel_imerode        = strel('disk',5);
kernel_connected_size = 8;

minimum_object_width = 1; % in meters
background_threshold = 37;

maximum_trackable_objects = 10; % Maximum trackable objects
max_label_distance = 5; % Maximum distance threshold

%%% Resolution Settings

% image_width = 1280;
% image_height = 720;
% image_height_buffer = 0;

image_width = 1920;
image_height = 1080;
image_height_buffer = 10;

%%% Blender settings

% f_x = 2008.8053  ;
% f_y = 2008.8053 ;
% p_x = 960.0000;
% p_y = 540.0000 ;
% s = 0;
% H = 2.5;

%%% Camera Settings -35mm

f_x = 3022.58829918849  ;
f_y = 3023.43980585740 ;
p_x = 972.352942326151;
p_y = 527.209473101826 ;
s = 0;
H = 1.3;

%%% Camera Settings -55mm

% f_x = 4609.85361371183  ;
% f_y = 4611.98648721446 ;
% p_x = 965.405810568760;
% p_y = 498.545149323953;
% s = 0;
% H = 1.3;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  PIXEL TO DISTANCE LOOK UP TABLE   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% PINHOLE MATRACIES
K = [f_x s p_x;0 f_y p_y;0 0 1];
K_inv = [f_y 0 (-p_x*f_y);0 f_x (-p_y*f_x);0 0 (f_x*f_y)]*1/((f_x).*(f_y));

image_to_distance_x= [];
image_to_distance_y= [];
image_to_distance_z= [];

% GENERATE LOOKUP TABLE
for i = 1:image_height/2-image_height_buffer
    for j = 1:image_width
        p_in = [j;i;1];
        
        rates = K_inv*p_in;
        parameter = -H/rates(2);
        p_out = rates*parameter;
        
        image_to_distance_x(i,j) = p_out(1);
        image_to_distance_y(i,j) = p_out(2);
        image_to_distance_z(i,j) = p_out(3);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Setup Video Reader/Writer  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%video_path = "OneVehicle/Rendered Animation/onevehiclerender.mp4";
%video_path = "OneVehicle/Rendered Animation/lane_switching.mp4";
%video_path = "OneVehicle/Rendered Animation/two_lanes.mp4";
%video_path = "OneVehicle/Rendered Animation/walking_lane_switching.mp4";
%video_path = "OneVehicle/Rendered Animation/lane_switching_single.mp4";
%video_path = "OneVehicle/Rendered Animation/Braking-1.m4v";
% video_path = "OneVehicle/Rendered Animation/short_Braking-1.mp4";
% video_path = "OneVehicle/Rendered Animation/Site4_Normal.mp4";
% video_path = "OneVehicle/Rendered Animation/Site4_SwitchingLanes.m4v";
video_path = "OneVehicle/Rendered Animation/Site3_Normal_Short.mp4";

video_input = VideoReader(video_path);

%video_writer = VideoWriter('Output/original_0p1.mp4','MPEG-4'); % Mac
video_writer = VideoWriter('Output/Site3_Normal_Short_acf');         % Linux

open(video_writer);

background = readFrame(video_input);
% background              = imread("OneVehicle/Background Image/0235.png");
% background              = imread("OneVehicle/Background Image/walking.png");
% background              = imread("OneVehicle/Background Image/Background.png");
% background              = imread("OneVehicle/Background Image/Site4_Normal_Background.png");
% background              = imread("OneVehicle/Background Image/Site4_SwitchingLanes_Background.png");

background_resized      = imresize(background,frame_scale_factor);
background_resized_gray = double(rgb2gray(background_resized));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Draw Background ROI       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

img = imshow(background_resized);
roi = images.roi.AssistedFreehand(img);
draw(roi)
mask = createMask(roi);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MATLAB Foreground Detector %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% detector =  vision.ForegroundDetector(...
%             'NumTrainingFrames', 150, ...
%             'InitialVariance', 30*30, ...
%             'NumGaussians',7);
        
detector = vehicleDetectorACF();

%%%%%%%%%%%%%%%%%%%%%%%%%
% Define runtime Arrays %
%%%%%%%%%%%%%%%%%%%%%%%%%

location_x = [];
location_y = [];

location_x_2 = [];
location_y_2 = [];

Objects_previous = zeros( 5, maximum_trackable_objects );
Objects_current  = zeros( 5, maximum_trackable_objects );
Distance_matrix  = zeros( maximum_trackable_objects , maximum_trackable_objects );

%%%%%%%%%%%%%%%%%%%%%%
%  MAIN VIDEO LOOP   %
%%%%%%%%%%%%%%%%%%%%%%

counter   = 1;
max_label = 0;

while hasFrame(video_input)
    
    %%% READ FRAME %%%
    frame_current = readFrame( video_input ); 
    frame_resized = imresize( frame_current,frame_scale_factor );
    
    % MASK IMAGE
    frame_resized = bsxfun(@times, frame_resized, cast(mask, 'like', frame_resized));
    
    % Save data from previous frame
    old_max_label    = max_label;
    Objects_previous = Objects_current;  
    Distance_matrix  = NaN(maximum_trackable_objects,maximum_trackable_objects);
    
    % Bounding box
    [bounding_box,scores] = detect(detector,frame_resized);
    
    max_label = length(scores); % Find max label
    
    number_removed_objects = 0 ; 
    for label_index = 1:max_label
        
        label_current = label_index - number_removed_objects; % Adjust for removed objects
        
        % bounding box --> General
        box_x1 = bounding_box(label_current,1);
        box_y1 = bounding_box(label_current,2);
        box_x2 = box_x1 + bounding_box(label_current,3);
        box_y2 = box_y1 + bounding_box(label_current,4);
        
        % co-ordinates adjusted for scale factor
        x1 = 1 + floor(box_x1/frame_scale_factor);  
        x2 = 1 + floor(box_x2/frame_scale_factor);
        y  = 1 + image_height -  floor( max( box_y1 ,box_y2 )/frame_scale_factor);
        
        % avoid above horizon 
        if(x1>image_width)                              x1 = image_width; end
        if(x2>image_width)                              x2 = image_width; end
        if(y>image_height/2 - image_height_buffer )     y  = image_height/2 - image_height_buffer; end
        if(y<1)                                         y  = 1; end
        
        X_1   = image_to_distance_x(y,x1);
        X_2   = image_to_distance_x(y,x2);
        Y     = image_to_distance_z(y,x1);
        Width = abs(X_1 - X_2);
        
        if Width > minimum_object_width && scores(label_current) > 15 % Remove object if its width is small
            
            Objects_current(1,label_current) = (X_1 + X_2)/2;  % Center x value
            Objects_current(2,label_current) = Y;              % Z distance away
            Objects_current(3,label_current) = Width;          % Width of car
            
            for old_object = 1: old_max_label
                Distance_matrix(label_current, old_object) = ...
                    (Objects_current(1,label_current) - Objects_previous(1,old_object))^2 ...
                     + (Objects_current(2,label_current)-Objects_previous(2,old_object))^2;
            end
            
            % Insert Shape
            frame_resized = insertShape(frame_resized,'Rectangle',[min(box_x1,box_x2) min(box_y1,box_y2) abs(box_x1 - box_x2) abs(box_y1 - box_y2)],'LineWidth',5,'Color','green');
%             frame_subtracted = insertShape(mat2gray(frame_subtracted),'Rectangle',[min(box_x1,box_x2) min(box_y1,box_y2) abs(box_x1 - box_x2) abs(box_y1 - box_y2)],'LineWidth',5,'Color','green');
            
        else
            number_removed_objects = number_removed_objects + 1;
        end
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
                location_y = [location_y Objects_current(2,label_current)];
                
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
        
    end
    
    if ~found_label_2
        
        location_x_2 = [location_x_2 NaN];
        location_y_2 = [location_y_2 NaN];
        
    end
    
    % Write Output Video    
    writeVideo(video_writer,frame_resized);
%     writeVideo(video_writer,mat2gray(frame_subtracted));
    
    counter = counter + 1
end

%location_total = transpose([location_x ;location_y]);
%location_total = transpose([location_x ;location_y;location_x_2 ;location_y_2]);

%writematrix(location_total,'Output/output_location.csv');

figure(1)
plot(location_x,location_y)
xlim([-10 10])

figure(2)
plot(location_x_2,location_y_2)
xlim([-10 10])

figure(3)
scatter(location_x,location_y,'r')
xlim([-10 10])
hold on
scatter(location_x_2,location_y_2,'b')
hold off

close(video_writer)