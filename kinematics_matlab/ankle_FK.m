clear all; close all; clc;

syms theta_1 theta_2 phi_y psi_x real
syms h_z_1 a_y_1 b_x_1 f_x_1 f_y_1 f_z_1 L_1 real
syms h_z_2 a_y_2 b_x_2 f_x_2 f_y_2 f_z_2 L_2 real

R_input_theta1 = [cos(theta_1) 0 sin(theta_1); 
          0 1 0; 
          -sin(theta_1) 0 cos(theta_1)];

R_input_theta2 = [cos(theta_2) 0 sin(theta_2); 
          0 1 0; 
          -sin(theta_2) 0 cos(theta_2)];
      
R_ankle_pitch =  [cos(phi_y) 0 sin(phi_y); 
          0 1 0; 
          -sin(phi_y) 0 cos(phi_y)];
      
R_ankle_roll = [1 0 0; 
          0 cos(psi_x) -sin(psi_x); 
          0 sin(psi_x) cos(psi_x)];

%% it takes 
clc; 
syms t_1 t_2 real % t_1 = tan(phi_y/2), t_2 = tan(psi_x/2)

% Define the vector equations V_1 and V_2
V_1 = [0; 0; h_z_1] + R_ankle_pitch * R_ankle_roll * [f_x_1; f_y_1; f_z_1] - [0; a_y_1; 0] - R_input_theta1 * [b_x_1; 0; 0];
V_2 = [0; 0; h_z_2] + R_ankle_pitch * R_ankle_roll * [f_x_2; f_y_2; f_z_2] - [0; a_y_2; 0] - R_input_theta2 * [b_x_2; 0; 0];

eq1 = simplify(expand(V_1.' * V_1 - L_1^2));
eq2 = simplify(expand(V_2.' * V_2 - L_2^2));

eq1_subs = subs(eq1, {cos(phi_y), sin(phi_y)}, {(1-t_1^2)/(1+t_1^2), 2*t_1/(1+t_1^2)}); % reparametrization cos, sin to tan(phi_y/2)
eq1_new = simplify(expand(eq1_subs*(1+t_1^2)));
[c1, eq1_terms] = coeffs(eq1_new, t_1);

eq2_subs = subs(eq2, {cos(phi_y), sin(phi_y)}, {(1-t_1^2)/(1+t_1^2), 2*t_1/(1+t_1^2)}); % reparametrization cos, sin to tan(phi_y/2)
eq2_new = simplify(expand(eq2_subs*(1+t_1^2)));
[c2, eq2_terms] = coeffs(eq2_new, t_1);

cross_product = cross(c1, c2);
decoupled_eq = expand(cross_product(1)*cross_product(3) -cross_product(2)^2); % decoupled eq for t_2
decoupled_eq_subs = subs(decoupled_eq, {cos(psi_x), sin(psi_x)}, {(1-t_2^2)/(1+t_2^2), 2*t_2/(1+t_2^2)});
decoupled_eq_new = simplify(expand(decoupled_eq_subs*(1+t_2^2)^4));
[coeffs, terms] = coeffs(decoupled_eq_new, t_2) % 8-th order equation for t_2


%% check (without f_z version)
% simplify(eq1 - (A_cos(1)*cos(phi_y)+B_sin(1)*sin(phi_y)+constant_terms)) % check zero

% eq3 = a_y_1^2 - L_1^2 + h_z_1^2 + b_x_1^2 + f_x_1^2 + f_y_1^2 + 2*b_x_1*h_z_1*sin(theta_1) -...
%     2*b_x_1*f_x_1*cos(phi_y)*cos(theta_1) - 2*b_x_1*f_x_1*sin(phi_y)*sin(theta_1) - 2*f_x_1*h_z_1*sin(phi_y) - ...
%     2*a_y_1*f_y_1*cos(psi_x) + ...
%     2*f_y_1*h_z_1*cos(phi_y)*sin(psi_x) + 2*b_x_1*f_y_1*cos(phi_y)*sin(psi_x)*sin(theta_1) - 2*b_x_1*f_y_1*sin(phi_y)*sin(psi_x)*cos(theta_1)
% simplify(eq1) - eq3

% [A_cos, term_cos] = coeffs(eq1, cos(phi_y))
% [B_sin, term_sin] = coeffs(eq1, sin(phi_y))
% constant_terms = subs(eq1, {cos(phi_y), sin(phi_y)}, {0, 0}); 

%%
load("computation_result_FK.mat")

%% test 

input = [0.5723, 0.29653];
temp_symbolic = {h_z_1, a_y_1, b_x_1, f_x_1, f_y_1,f_z_1, L_1, h_z_2, a_y_2, b_x_2, f_x_2, f_y_2, f_z_2, L_2, theta_1, theta_2}
temp_real = {-0.205, 0.0405, 0.065, -0.0640125, 0.04, 0.0112871, 0.198, -0.32, -0.0405, 0.065, -0.0640125, -0.04, 0.0112871, 0.316,input(1), input(2)}
real_coeffs = subs(coeffs, temp_symbolic, temp_real)*1e6;
r = roots(real_coeffs);
real_roots = r(imag(r) == 0)
psi_x_set = atan(real_roots)*2
psi_x_set=psi_x_set(abs(psi_x_set)<1.0); % heuristics, 설계상 1 rad 넘지 못함 pitch, roll 둘다 (needed)

solution_set = zeros(2,length(psi_x_set)*2);
for i = 1:length(psi_x_set)
    real_c1 = subs(c1,temp_symbolic, temp_real)*1e2;
    real_c1 = subs(real_c1,{psi_x},psi_x_set(i));
    r = roots(real_c1);
    real_roots = r(imag(r) == 0);
    phi_y_set = atan(real_roots)*2;
%     [~,ind] = sort(abs(phi_y_set));
%     phi_y_set = phi_y_set(ind);
    solution_set(:,i*2-1) = [phi_y_set(1);psi_x_set(i)]; 
    solution_set(:,i*2) = [phi_y_set(2);psi_x_set(i)]; 
end
solution_set = solution_set(:,abs(solution_set(1,:))<1); % heuristics, 설계상 1 rad 넘지 못함 pitch, roll 둘다 (needed)

% validation eq
eq1_validation = subs(eq1, temp_symbolic, temp_real);
eq2_validation = subs(eq2, temp_symbolic, temp_real);
num_sol = 0;

for i= 1:length(solution_set)
    eq1_temp = double(subs(eq1_validation, {phi_y, psi_x},{solution_set(1,i),solution_set(2,i)}));
    eq2_temp = double(subs(eq2_validation, {phi_y, psi_x},{solution_set(1,i),solution_set(2,i)}));
    if (abs(eq1_temp) < 1e-10 && abs(eq2_temp) < 1e-10)
        double_phi_y = solution_set(1,i); % heuristics
        double_psi_x = solution_set(2,i); % heuristics
        num_sol = num_sol + 1;
    end
end
if (num_sol ~= 1)
    disp("no sol or too many sol");
else
    clc;
    disp("input angle");
    disp(input)
    disp("solution");
    disp([double_phi_y,double_psi_x]);
end

% real_c1 = subs(c1,temp_symbolic, temp_real)*1e2;
% real_c1 = subs(real_c1,{psi_x},psi_x_set(1))
% r = roots(real_c1);
% real_roots = r(imag(r) == 0)
% phi_y_set = atan(real_roots)*2
%% compute Jacobian -> implicit function theorem
% F(x,y)=0 -> dy/dx = -inv(dF/dy)*(dF/dx)

dF_dx = [diff(eq1,theta_1), 0; 0, diff(eq2,theta_2)];
dF_dy = [diff(eq1,phi_y),diff(eq1,psi_x);diff(eq2,phi_y),diff(eq2,psi_x)];

% test
temp_symbolic = {h_z_1, a_y_1, b_x_1, f_x_1, f_y_1, f_z_1, L_1, h_z_2, a_y_2, b_x_2, f_x_2, f_y_2, f_z_2, L_2, theta_1, theta_2, phi_y, psi_x}
temp_real = {-0.205, 0.0405, 0.065, -0.0640125, 0.04, 0.0112871, 0.198, -0.32, -0.0405, 0.065, -0.0640125, -0.04, 0.0112871, 0.316, 0.5723, 0.29653, double_phi_y, double_psi_x}
real_dF_dx = subs(dF_dx,temp_symbolic, temp_real)
real_dF_dy = subs(dF_dy,temp_symbolic, temp_real)
dy_dx = -double(real_dF_dy\real_dF_dx)
