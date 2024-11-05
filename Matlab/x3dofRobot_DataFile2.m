% Simscape(TM) Multibody(TM) version: 7.6

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(9).translation = [0.0 0.0 0.0];
smiData.RigidTransform(9).angle = 0.0;
smiData.RigidTransform(9).axis = [0.0 0.0 0.0];
smiData.RigidTransform(9).ID = "";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [7.8740157480338393 -0.57741971665882974 0.2962044222403768];  % in
smiData.RigidTransform(1).angle = 2.0943951023923528;  % rad
smiData.RigidTransform(1).axis = [-0.57735026918934484 -0.57735026919035204 -0.57735026918918031];
smiData.RigidTransform(1).ID = "B[Link2-1:-:Link3.step-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [2.72759694154424 -0.086825389111718221 0.33495644136306035];  % in
smiData.RigidTransform(2).angle = 2.0943951023920504;  % rad
smiData.RigidTransform(2).axis = [-0.57735026918924393 -0.57735026918924437 -0.57735026919038901];
smiData.RigidTransform(2).ID = "F[Link2-1:-:Link3.step-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [0.081299208445345744 -0.43307086614173484 0.391732283460343];  % in
smiData.RigidTransform(3).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(3).axis = [-0.57735026918962584 -0.57735026918962573 -0.57735026918962573];
smiData.RigidTransform(3).ID = "B[khau_xoay_link12.step-1:-:Link2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [-1.7249535133601057e-12 1.7847850077369418 0.29620442222837079];  % in
smiData.RigidTransform(4).angle = 2.0943951023923542;  % rad
smiData.RigidTransform(4).axis = [-0.57735026918934529 -0.57735026919035259 -0.57735026918917931];
smiData.RigidTransform(4).ID = "F[khau_xoay_link12.step-1:-:Link2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [0 0.098425196846903582 3.8267716535433074];  % in
smiData.RigidTransform(5).angle = 3.1415926535887557;  % rad
smiData.RigidTransform(5).axis = [-1 -1.405891243584917e-28 2.7105054312165798e-16];
smiData.RigidTransform(5).ID = "B[Base-1:-:khau_xoay_link12.step-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(6).translation = [-0.25728346858589224 -1.7716535433074989 -2.7854330708660404];  % in
smiData.RigidTransform(6).angle = 3.1415926535879128;  % rad
smiData.RigidTransform(6).axis = [-1 1.1762554227910593e-30 -1.2511240570693969e-18];
smiData.RigidTransform(6).ID = "F[Base-1:-:khau_xoay_link12.step-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(7).translation = [1.0563931613204225 -1.1642983848628099 -2.7559055118110249];  % in
smiData.RigidTransform(7).angle = 2.0893641201285913;  % rad
smiData.RigidTransform(7).axis = [-0.57566348316160632 -0.5807091426079809 -0.57566348316160432];
smiData.RigidTransform(7).ID = "B[khau_link2_noidc.step-1:-:Link2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(8).translation = [2.9473229412602109e-12 -0.89238034659376353 0.29620442222914412];  % in
smiData.RigidTransform(8).angle = 2.0943951023923533;  % rad
smiData.RigidTransform(8).axis = [-0.57735026918934518 -0.57735026919035248 -0.57735026918917975];
smiData.RigidTransform(8).ID = "F[khau_link2_noidc.step-1:-:Link2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(9).translation = [-8.9388683581804322 9.9556420936300434 5.9096272367984941];  % in
smiData.RigidTransform(9).angle = 0;  % rad
smiData.RigidTransform(9).axis = [0 0 0];
smiData.RigidTransform(9).ID = "RootGround[Base-1]";


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(5).mass = 0.0;
smiData.Solid(5).CoM = [0.0 0.0 0.0];
smiData.Solid(5).MoI = [0.0 0.0 0.0];
smiData.Solid(5).PoI = [0.0 0.0 0.0];
smiData.Solid(5).color = [0.0 0.0 0.0];
smiData.Solid(5).opacity = 0.0;
smiData.Solid(5).ID = "";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 0.55504291982628284;  % kg
smiData.Solid(1).CoM = [3.3928335602799963e-05 2.6042089298362483 34.482557169530743];  % mm
smiData.Solid(1).MoI = [1486.0081529958547 1487.3655511956583 1684.2472204081234];  % kg*mm^2
smiData.Solid(1).PoI = [1.1692735437097397 -0.0011220907017472196 0.00099767495396151833];  % kg*mm^2
smiData.Solid(1).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = "Base*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 0.089489382385736393;  % kg
smiData.Solid(2).CoM = [132.21344147223965 27.794635116547621 16.0357717448132];  % mm
smiData.Solid(2).MoI = [57.716470416188997 656.52085439096413 688.80433146007545];  % kg*mm^2
smiData.Solid(2).PoI = [0 43.501660606163412 0];  % kg*mm^2
smiData.Solid(2).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = "Link3.step*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 0.03277637411348016;  % kg
smiData.Solid(3).CoM = [0.97810752691042291 -0.2274927126235495 -2.0479215747786217];  % in
smiData.Solid(3).MoI = [0.059419655755645125 0.041832457357090735 0.024950899633931392];  % kg*in^2
smiData.Solid(3).PoI = [-0.0011698878386060534 0.0016084441955579228 0.00026104690989882726];  % kg*in^2
smiData.Solid(3).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = "khau_link2_noidc.step*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(4).mass = 0.43416950903050766;  % kg
smiData.Solid(4).CoM = [-0.47007019264927458 -44.184719402875153 -10.771837756065761];  % mm
smiData.Solid(4).MoI = [2436.144520960067 661.46115206189938 2250.8152751107141];  % kg*mm^2
smiData.Solid(4).PoI = [-10.00130521594345 -46.721012441081257 1.1718262952707834];  % kg*mm^2
smiData.Solid(4).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(4).opacity = 1;
smiData.Solid(4).ID = "khau_xoay_link12.step*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(5).mass = 0.08903783961947298;  % kg
smiData.Solid(5).CoM = [86.063175044451086 16.724002889649601 0.10926245254135504];  % mm
smiData.Solid(5).MoI = [84.639502128857259 400.34769694796972 451.69015473284429];  % kg*mm^2
smiData.Solid(5).PoI = [-0.91792283014891973 6.1097750523846175 10.654955278450103];  % kg*mm^2
smiData.Solid(5).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(5).opacity = 1;
smiData.Solid(5).ID = "Link2*:*Default";


%============= Joint =============%
%X Revolute Primitive (Rx) %Y Revolute Primitive (Ry) %Z Revolute Primitive (Rz)
%X Prismatic Primitive (Px) %Y Prismatic Primitive (Py) %Z Prismatic Primitive (Pz) %Spherical Primitive (S)
%Constant Velocity Primitive (CV) %Lead Screw Primitive (LS)
%Position Target (Pos)

%Initialize the RevoluteJoint structure array by filling in null values.
smiData.RevoluteJoint(4).Rz.Pos = 0.0;
smiData.RevoluteJoint(4).ID = "";

smiData.RevoluteJoint(1).Rz.Pos = 48.347446635367753;  % deg
smiData.RevoluteJoint(1).ID = "[Link2-1:-:Link3.step-1]";

smiData.RevoluteJoint(2).Rz.Pos = -12.717094343049082;  % deg
smiData.RevoluteJoint(2).ID = "[khau_xoay_link12.step-1:-:Link2-1]";

smiData.RevoluteJoint(3).Rz.Pos = -10.090564682882297;  % deg
smiData.RevoluteJoint(3).ID = "[Base-1:-:khau_xoay_link12.step-1]";

smiData.RevoluteJoint(4).Rz.Pos = 164.22499933099809;  % deg
smiData.RevoluteJoint(4).ID = "[khau_link2_noidc.step-1:-:Link2-1]";

