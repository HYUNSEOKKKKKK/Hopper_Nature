filePath = 'URDF_AutoGenerator_root.txt';
fileID = fopen(filePath, 'r');
textData = fread(fileID, '*char')';
fclose(fileID);

%% Trunk parameter setting.

% Inertia.
textData = strrep(textData, 'trunk_CoM_x', sprintf('%.8f', 0.001319));
textData = strrep(textData, 'trunk_CoM_y', sprintf('%.8f', 0.000239));
textData = strrep(textData, 'trunk_CoM_z', sprintf('%.8f', 0.003879));

textData = strrep(textData, 'trunk_mass', sprintf('%.8f', 18.476235));
textData = strrep(textData, 'trunk_inertia_ixx', sprintf('%.8f', 0.217213));
textData = strrep(textData, 'trunk_inertia_ixy', sprintf('%.8f', 0.001430));
textData = strrep(textData, 'trunk_inertia_ixz', sprintf('%.8f', 0.002076));
textData = strrep(textData, 'trunk_inertia_iyy', sprintf('%.8f', 0.803514));
textData = strrep(textData, 'trunk_inertia_iyz', sprintf('%.8f', 0.000309));
textData = strrep(textData, 'trunk_inertia_izz', sprintf('%.8f', 0.958084));


% Collision body.
textData = strrep(textData, 'trunk_collision_length_x', sprintf('%.8f', 0.562));
textData = strrep(textData, 'trunk_collision_length_y', sprintf('%.8f', 0.251));
textData = strrep(textData, 'trunk_collision_length_z', sprintf('%.8f', 0.140));

%% Roll joint parameter setting.


% joint position.
textData = strrep(textData, 'roll_joint_x', sprintf('%.8f', 0.351470));
textData = strrep(textData, 'roll_joint_y', sprintf('%.8f', 0.100000));
textData = strrep(textData, 'roll_joint_z', sprintf('%.8f', 0.000000));


% Inertia.
textData = strrep(textData, 'roll_rotor_inertia', sprintf('%.8f', 0.014056));

textData = strrep(textData, 'shoulder_mass', sprintf('%.8f', 3.86933709));

textData = strrep(textData, 'shoulder_CoM_x', sprintf('%.8f', -0.00304326));
textData = strrep(textData, 'shoulder_CoM_y', sprintf('%.8f', -0.00050825));
textData = strrep(textData, 'shoulder_CoM_z', sprintf('%.8f', -0.00001789));

textData = strrep(textData, 'shoulder_inertia_ixx', sprintf('%.8f', 0.00858804));
textData = strrep(textData, 'shoulder_inertia_ixy', sprintf('%.8f', 0.00015474));
textData = strrep(textData, 'shoulder_inertia_ixz', sprintf('%.8f', -0.00000061));
textData = strrep(textData, 'shoulder_inertia_iyy', sprintf('%.8f', 0.00737145));
textData = strrep(textData, 'shoulder_inertia_iyz', sprintf('%.8f', 0.00000042));
textData = strrep(textData, 'shoulder_inertia_izz', sprintf('%.8f', 0.00928864));


% Acutator limit.
textData = strrep(textData, 'roll_max_torque', sprintf('%.8f', 200));
textData = strrep(textData, 'roll_max_speed', sprintf('%.8f', 21.29));
textData = strrep(textData, 'roll_angle', sprintf('%.8f', 45*(pi/180)));

% Collision body.
textData = strrep(textData, 'shoulder_collision_cylinder_radius', sprintf('%.8f', 0.061));
textData = strrep(textData, 'shoulder_collision_cylinder_length', sprintf('%.8f', 0.133));
textData = strrep(textData, 'shoulder_collision_cylinder_offset', sprintf('%.8f', 0.005));


%% Hip joint parameter setting.

% Joint position.
textData = strrep(textData, 'hip_joint_y', sprintf('%.8f', 0.1135));

% Inertia.
textData = strrep(textData, 'hip_rotor_inertia', sprintf('%.8f', 0.014056));

textData = strrep(textData, 'thigh_mass', sprintf('%.8f', 1.3877582));

textData = strrep(textData, 'thigh_CoM_x', sprintf('%.8f', 0.0));
textData = strrep(textData, 'thigh_CoM_y', sprintf('%.8f', 0.01475079));
textData = strrep(textData, 'thigh_CoM_z', sprintf('%.8f', -0.11906126));

textData = strrep(textData, 'thigh_inertia_ixx', sprintf('%.8f', 0.02401331));
textData = strrep(textData, 'thigh_inertia_ixy', sprintf('%.8f', 0.0));
textData = strrep(textData, 'thigh_inertia_ixz', sprintf('%.8f', 0.0));
textData = strrep(textData, 'thigh_inertia_iyy', sprintf('%.8f', 0.02340658));
textData = strrep(textData, 'thigh_inertia_iyz', sprintf('%.8f', 0.00229407));
textData = strrep(textData, 'thigh_inertia_izz', sprintf('%.8f', 0.00240062));


% Acutator limit.
textData = strrep(textData, 'hip_max_torque', sprintf('%.8f', 200));
textData = strrep(textData, 'hip_max_speed', sprintf('%.8f', 21.29));
textData = strrep(textData, 'hip_angle', sprintf('%.8f', 360*(pi/180)));

% Collision body.
textData = strrep(textData, 'thigh_collision_box_x', sprintf('%.8f', 0.085));
textData = strrep(textData, 'thigh_collision_box_y', sprintf('%.8f', 0.070));
textData = strrep(textData, 'thigh_collision_box_z', sprintf('%.8f', 0.350));
textData = strrep(textData, 'thigh_collision_box_offset', sprintf('%.8f', -0.16395));

%% Knee joint parameter setting.

% Joint position.
textData = strrep(textData, 'knee_joint_z', sprintf('%.8f', -0.3279)); %% Length of Thigh: Hip-to-Knee length.

textData = strrep(textData, 'calf_mass', sprintf('%.8f', 0.82426578));

textData = strrep(textData, 'calf_CoM_x', sprintf('%.8f', 0.0));
textData = strrep(textData, 'calf_CoM_y', sprintf('%.8f', 0.0));
textData = strrep(textData, 'calf_CoM_z', sprintf('%.8f', -0.15092214));

textData = strrep(textData, 'calf_inertia_ixx', sprintf('%.8f', 0.01867598));
textData = strrep(textData, 'calf_inertia_ixy', sprintf('%.8f', 0.0));
textData = strrep(textData, 'calf_inertia_ixz', sprintf('%.8f', 0.0));
textData = strrep(textData, 'calf_inertia_iyy', sprintf('%.8f', 0.01879191));
textData = strrep(textData, 'calf_inertia_iyz', sprintf('%.8f', 0.00000684));
textData = strrep(textData, 'calf_inertia_izz', sprintf('%.8f', 0.00058083));


% Acutator limit.
textData = strrep(textData, 'knee_max_torque', sprintf('%.8f', 400));
textData = strrep(textData, 'knee_max_speed', sprintf('%.8f', 10.00));
textData = strrep(textData, 'knee_angle', sprintf('%.8f', 160*(pi/180)));

% Collision body.
textData = strrep(textData, 'calf_collision_cylinder_radius', sprintf('%.8f', 0.025));
textData = strrep(textData, 'calf_collision_cylinder_length', sprintf('%.8f', 0.265));
textData = strrep(textData, 'calf_collision_cylinder_offset', sprintf('%.8f', -0.200));


%% Foot parameter setting.

textData = strrep(textData, 'foot_joint_z', sprintf('%.8f', -0.350)); %% Length of Calf: Knee-to-Foot length.
textData = strrep(textData, 'foot_collision_sphere_radius', sprintf('%.8f', 0.038));

%% Remove "--"
textData = strrep(textData, '<!--', 'd45v1');
textData = strrep(textData, '-->', '5nv67');

textData = strrep(textData, '--', '');

textData = strrep(textData, 'd45v1', '<!--');
textData = strrep(textData, '5nv67', '-->');

%% Save.

newFilePath = 'HoundOne_generated.urdf';
fileID = fopen(newFilePath, 'w');
fwrite(fileID, textData, 'char');
fclose(fileID);

disp('New URDF file generated.');
