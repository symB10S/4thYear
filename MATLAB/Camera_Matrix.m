f_x = 2008.8053  ;
f_y = 2008.8053 ;
p_x = 960.0000;
p_y = 540.0000 ;
s = 0;

image_width = 1920;
image_height = 1080;

K = [f_x s p_x;0 f_y p_y;0 0 1];
K_inv = [f_y 0 (-p_x*f_y);0 f_x (-p_y*f_x);0 0 (f_x*f_y)]*1/((f_x).*(f_y));


theta = 0;
R = [1 0 0; 0 cos(theta) sin(theta); 0 -sin(theta) cos(theta)];

%=== DEFINE A RECTANGLE ===%
%Car Parameters
%Length     =  4.36m
%Width         =  1.74m
%Height         =  1.46m


W = 1.74;    %Width
L = 4.36;     %Lenght
T = 1.46;    %Tallness

H = 2.5;  %Camera Height 
D = 24.63; %Distance from Camera

delta_x = -2.03;
delta_y = -H;
delta_z = 0;

p_1 = [-W/2;T;D]+[delta_x;delta_y;delta_z];
p_2 = [W/2;T;D]+[delta_x;delta_y;delta_z];
p_3 = [W/2;0;D]+[delta_x;delta_y;delta_z];
p_4 = [-W/2;0;D]+[delta_x;delta_y;delta_z];
p_5 = p_1 + [0;0;L];
p_6 = p_2 + [0;0;L];
p_7 = p_3 + [0;0;L];
p_8 = p_4 + [0;0;L];

%Transform Points Individually 3D -> 2D
P_1 = K*p_1;
P_2 = K*p_2;
P_3 = K*p_3;
P_4 = K*p_4;
P_5 = K*p_5;
P_6 = K*p_6;
P_7 = K*p_7;
P_8 = K*p_8;

%Alternate Description
p_array = [p_1 p_2 p_3 p_4 p_1 p_5 p_6 p_7 p_8 p_5 p_6 p_2 p_3 p_7 p_8 p_4];

out = [];
for I = p_array
    out = [out K*R*I];
end


image = imread("OneVehicle/Rendered Images/0185.png");
image_flipped = flip(image ,1);     

figure(1)
imshow(image_flipped)
%imshow("OneVehicle/Rendered Images/0200.png")
%imshow("scaled_1.png")
hold on
plot(out(1,:)./out(3,:),out(2,:)./out(3,:))
axis([0 image_width 0 image_height])

%=== DEFINE A ROAD ===%
W = 1 ;
L = 10 ;
D = 2 ;
H = 1 ;

p_11 = [-W;-H;D];
p_12 = [-W;-H;D+L];
p_21 = [W;-H;D];
p_22 = [W;-H;D+L];

road_path = [p_11 p_12 p_22 p_21 p_11 p_22 p_12 p_21];

out = [];
for I = road_path
    out = [out K*I];
end

figure(2)
plot(out(1,:)./out(3,:),out(2,:)./out(3,:))
axis([0 image_width 0 image_height])


%=== DEFINE ZIG-ZAG ROAD ===%
W = 1 ;
D = 2 ;
H = 2.5 ;
S = 0.2;
number_zigs = 20;

zig_zag_path = [];
p_start = [-W; -H; D];
for i = 1:number_zigs
    p_a = p_start + [0;0;S];
    p_b = p_a + [2*W;0;0];
    p_c = p_b + [0;0;S];
    p_d = p_c - [2*W;0;0];
    
    zig_zag_path = [zig_zag_path p_start p_a p_b p_c p_d];
    p_start = p_d;
end

out = [];
for I = zig_zag_path
    out = [out K*I];
end

figure(3)
plot(out(1,:)./out(3,:),out(2,:)./out(3,:))
axis([0 image_width 0 image_height])


% === INVERSE CAMERA MATRIX === %

p_in = out(:,2)/out(3,2);

rates = K_inv*p_in;
parameter = -H/rates(2);
p_out = rates*parameter

% Generate Perspective for lower than Horizon
blank_image = zeros(image_height,image_width,3,'uint8');
blank_image_x= [];
blank_image_y= [];
blank_image_z= [];
blank_image_abs= [];

for i = 1:image_height/2-100
    for j = 1:image_width
        p_in = [j;i;1];
        
        rates = K_inv*p_in;
        parameter = -H/rates(2);
        p_out = rates*parameter;
        
        blank_image_x(i,j) = p_out(1);
        blank_image_y(i,j) = p_out(2);
        blank_image_z(i,j) = p_out(3);
        
        blank_image_abs(i,j)= sqrt(p_out(1)*p_out(1)+p_out(2)*p_out(2)+p_out(3)*p_out(3));
        %image(round(p_out(1)),round(p_out(3)),:) = image_flipped(i,j,:);
    end
end

figure(4)
h = surf(blank_image_x)
set(h,'LineStyle','none')

figure(5)
h = surf(blank_image_y)
set(h,'LineStyle','none')

figure(6)
h = surf(blank_image_z)
set(h,'LineStyle','none')

figure(7)
h = surf(blank_image_abs)
set(h,'LineStyle','none')
        


