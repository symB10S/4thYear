f_x = 1000;
f_y = 1000;
p_x = 600;
p_y = 600;
s = 0;

K = [f_x s p_x;0 f_y p_y;0 0 1];
K_inv = [f_y 0 (-p_x*f_y);0 f_x (-p_y*f_x);0 0 (f_x*f_y)]*1/((f_x).*(f_y));


theta = 0;
R = [1 0 0; 0 cos(theta) sin(theta); 0 -sin(theta) cos(theta)];

%=== DEFINE A CUBE ===%
L = 1;
H = 1;
D = 4;

delta_x = L/2;
delta_y = -H/2;
delta_z = 0;

p_1 = [0;L;D]+[delta_x;delta_y;delta_z];
p_2 = [L;L;D]+[delta_x;delta_y;delta_z];
p_3 = [L;0;D]+[delta_x;delta_y;delta_z];
p_4 = [0;0;D]+[delta_x;delta_y;delta_z];
p_5 = p_1 + [0;0;H];
p_6 = p_2 + [0;0;H];
p_7 = p_3 + [0;0;H];
p_8 = p_4 + [0;0;H];

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

figure(1)
plot(out(1,:)./out(3,:),out(2,:)./out(3,:))
axis([0 1200 0 1200])

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
axis([0 1200 0 1200])


%=== DEFINE ZIG-ZAG ROAD ===%
W = 1 ;
D = 2 ;
H = 2 ;
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
axis([0 1200 0 1200])


% === INVERSE CAMERA MATRIX === %

p_in = out(:,2)/out(3,2);

rates = K_inv*p_in;
parameter = -H/rates(2);
p_out = rates*parameter





