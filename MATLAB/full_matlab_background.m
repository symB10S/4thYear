videoSource = VideoReader('OneVehicle/Rendered Animation/Site4_SwitchingLanes.m4v');
video_writer = VideoWriter('Output/matlab_background_subtraction');
background = imread("OneVehicle/Background Image/Site4_SwitchingLanes_Background.png");
open(video_writer);

img = imshow(background);
roi = images.roi.AssistedFreehand(img);
draw(roi)
mask = createMask(roi);

detector =  vision.ForegroundDetector(...
            'NumTrainingFrames', 150, ...
            'InitialVariance', 30*30);
     
blob = vision.BlobAnalysis(...
       'CentroidOutputPort', false, 'AreaOutputPort', false, ...
       'BoundingBoxOutputPort', true, ...
       'MinimumBlobAreaSource', 'Property', 'MinimumBlobArea', 250);
   
shapeInserter = vision.ShapeInserter('BorderColor','White');

counter = 0;

while hasFrame(videoSource)
     frame  = readFrame(videoSource);
%      frame(~mask) = 0;
     frame = bsxfun(@times, frame, cast(mask, 'like', frame));
     
     fgMask = detector(frame);
     bbox   = blob(fgMask);
     out    = shapeInserter(frame,bbox);
     writeVideo(video_writer,out);
     
     counter = counter +1 
end

close(video_writer)