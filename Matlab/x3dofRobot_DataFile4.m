% Simscape(TM) Multibody(TM) version: 7.6

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(7).translation = [0.0 0.0 0.0];
smiData.RigidTransform(7).angle = 0.0;
smiData.RigidTransform(7).axis = [0.0 0.0 0.0];
smiData.RigidTransform(7).ID = "";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [7.874015748033842 -0.57741971665882885 0.29620442224037569];  % in
smiData.RigidTransform(1).angle = 2.0943951023923528;  % rad
smiData.RigidTransform(1).axis = [-0.57735026918934484 -0.57735026919035215 -0.57735026918918042];
smiData.RigidTransform(1).ID = "B[Link2-1:-:Link3.step-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [2.7275969415440748 -0.086825389111698126 0.33495644136322211];  % in
smiData.RigidTransform(2).angle = 2.0943951023920513;  % rad
smiData.RigidTransform(2).axis = [-0.57735026918924448 -0.57735026918924381 -0.57735026919038912];
smiData.RigidTransform(2).ID = "F[Link2-1:-:Link3.step-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [0.081299208445346841 -0.43307086614173368 0.391732283460343];  % in
smiData.RigidTransform(3).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(3).axis = [-0.57735026918962584 -0.57735026918962573 -0.57735026918962573];
smiData.RigidTransform(3).ID = "B[khau_xoay_link12.step-1:-:Link2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [-1.815880779076906e-12 1.7847850077368963 0.29620442222823889];  % in
smiData.RigidTransform(4).angle = 2.0943951023923546;  % rad
smiData.RigidTransform(4).axis = [-0.57735026918934551 -0.57735026919035137 -0.57735026918918042];
smiData.RigidTransform(4).ID = "F[khau_xoay_link12.step-1:-:Link2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [0 0.098425196846903582 3.8267716535433074];  % in
smiData.RigidTransform(5).angle = 3.1415926535887557;  % rad
smiData.RigidTransform(5).axis = [-1 -1.405891243584917e-28 2.7105054312165798e-16];
smiData.RigidTransform(5).ID = "B[Base-1:-:khau_xoay_link12.step-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(6).translation = [-0.2572834685858818 -1.7716535433074827 -2.785433070866036];  % in
smiData.RigidTransform(6).angle = 3.1415926535879128;  % rad
smiData.RigidTransform(6).axis = [-1 -4.6331416374133416e-29 4.9275296306172954e-17];
smiData.RigidTransform(6).ID = "F[Base-1:-:khau_xoay_link12.step-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(7).translation = [-8.9388683581804322 9.9556420936300434 5.9096272367984941];  % in
smiData.RigidTransform(7).angle = 0;  % rad
smiData.RigidTransform(7).axis = [0 0 0];
smiData.RigidTransform(7).ID = "RootGround[Base-1]";


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(4).mass = 0.0;
smiData.Solid(4).CoM = [0.0 0.0 0.0];
smiData.Solid(4).MoI = [0.0 0.0 0.0];
smiData.Solid(4).PoI = [0.0 0.0 0.0];
smiData.Solid(4).color = [0.0 0.0 0.0];
smiData.Solid(4).opacity = 0.0;
smiData.Solid(4).ID = "";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 0.43416950903050766;  % kg
smiData.Solid(1).CoM = [-0.47007019264927469 -44.184719402875153 -10.771837756065759];  % mm
smiData.Solid(1).MoI = [2436.1445209600665 661.46115206189927 2250.815275110715];  % kg*mm^2
smiData.Solid(1).PoI = [-10.001305215943443 -46.721012441081257 1.1718262952707816];  % kg*mm^2
smiData.Solid(1).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = "khau_xoay_link12.step*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 0.55504291982628295;  % kg
smiData.Solid(2).CoM = [3.3928335602763127e-05 2.604208929836247 34.482557169530736];  % mm
smiData.Solid(2).MoI = [1486.0081529958552 1487.3655511956595 1684.2472204081241];  % kg*mm^2
smiData.Solid(2).PoI = [1.1692735437097417 -0.001122090701747402 0.00099767495396107012];  % kg*mm^2
smiData.Solid(2).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = "Base*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 0.08903783961947298;  % kg
smiData.Solid(3).CoM = [86.063175044451114 16.724002889649601 0.10926245254135453];  % mm
smiData.Solid(3).MoI = [84.639502128857288 400.34769694796978 451.69015473284441];  % kg*mm^2
smiData.Solid(3).PoI = [-0.91792283014892051 6.1097750523846193 10.654955278450103];  % kg*mm^2
smiData.Solid(3).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = "Link2*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(4).mass = 0.089489382385736393;  % kg
smiData.Solid(4).CoM = [132.21344147223965 27.794635116547621 16.0357717448132];  % mm
smiData.Solid(4).MoI = [57.716470416188997 656.52085439096413 688.80433146007545];  % kg*mm^2
smiData.Solid(4).PoI = [0 43.501660606163412 0];  % kg*mm^2
smiData.Solid(4).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(4).opacity = 1;
smiData.Solid(4).ID = "Link3.step*:*Default";


%============= Joint =============%
%X Revolute Primitive (Rx) %Y Revolute Primitive (Ry) %Z Revolute Primitive (Rz)
%X Prismatic Primitive (Px) %Y Prismatic Primitive (Py) %Z Prismatic Primitive (Pz) %Spherical Primitive (S)
%Constant Velocity Primitive (CV) %Lead Screw Primitive (LS)
%Position Target (Pos)

%Initialize the RevoluteJoint structure array by filling in null values.
smiData.RevoluteJoint(3).Rz.Pos = 0.0;
smiData.RevoluteJoint(3).ID = "";

smiData.RevoluteJoint(1).Rz.Pos = 62.306920139619017;  % deg
smiData.RevoluteJoint(1).ID = "[Link2-1:-:Link3.step-1]";

smiData.RevoluteJoint(2).Rz.Pos = -42.83679912241324;  % deg
smiData.RevoluteJoint(2).ID = "[khau_xoay_link12.step-1:-:Link2-1]";

smiData.RevoluteJoint(3).Rz.Pos = -137.67747133636766;  % deg
smiData.RevoluteJoint(3).ID = "[Base-1:-:khau_xoay_link12.step-1]";

