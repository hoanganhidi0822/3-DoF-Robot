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
smiData.RigidTransform(1).translation = [7.8740157480338393 -0.5774197166588293 0.29620442224037463];  % in
smiData.RigidTransform(1).angle = 2.0943951023923524;  % rad
smiData.RigidTransform(1).axis = [-0.57735026918934496 -0.57735026919035193 -0.57735026918918042];
smiData.RigidTransform(1).ID = "B[Link2-1:-:Link3.step-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [2.7275969415443062 -0.086825389111715001 0.33495644136320912];  % in
smiData.RigidTransform(2).angle = 2.0943951023920517;  % rad
smiData.RigidTransform(2).axis = [-0.57735026918924448 -0.5773502691892447 -0.57735026919038801];
smiData.RigidTransform(2).ID = "F[Link2-1:-:Link3.step-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [11.354156174696454 2.0785289415969497 0.33531971266687899];  % in
smiData.RigidTransform(3).angle = 2.094395102394341;  % rad
smiData.RigidTransform(3).axis = [0.57735026919000765 -0.57735026919000754 0.57735026918886201];
smiData.RigidTransform(3).ID = "B[Link3.step-1:-:EndEffector-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [1.825738188972073 1.2192928705909793 0.65802810912405241];  % in
smiData.RigidTransform(4).angle = 2.1146643196567769;  % rad
smiData.RigidTransform(4).axis = [0.58395170533568985 -0.58395170533281471 0.56391560687255071];
smiData.RigidTransform(4).ID = "F[Link3.step-1:-:EndEffector-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [0.081299208445349033 -0.43307086614173296 0.39173228346034517];  % in
smiData.RigidTransform(5).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(5).axis = [-0.57735026918962584 -0.57735026918962573 -0.57735026918962573];
smiData.RigidTransform(5).ID = "B[khau_xoay_link12.step-1:-:Link2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(6).translation = [-1.7850165789923267e-12 1.7847850077368868 0.29620442222834914];  % in
smiData.RigidTransform(6).angle = 2.0943951023923537;  % rad
smiData.RigidTransform(6).axis = [-0.57735026918934518 -0.57735026919035193 -0.5773502691891802];
smiData.RigidTransform(6).ID = "F[khau_xoay_link12.step-1:-:Link2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(7).translation = [0 0.098425196846903582 3.8267716535433074];  % in
smiData.RigidTransform(7).angle = 3.1415926535887557;  % rad
smiData.RigidTransform(7).axis = [-1 -1.405891243584917e-28 2.7105054312165798e-16];
smiData.RigidTransform(7).ID = "B[Base-1:-:khau_xoay_link12.step-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(8).translation = [-0.25728346858588425 -1.7716535433074863 -2.7854330708660355];  % in
smiData.RigidTransform(8).angle = 3.1415926535879128;  % rad
smiData.RigidTransform(8).axis = [-1 -4.772968517322021e-29 5.0762524336367213e-17];
smiData.RigidTransform(8).ID = "F[Base-1:-:khau_xoay_link12.step-1]";

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
smiData.Solid(1).mass = 0.43416950903050772;  % kg
smiData.Solid(1).CoM = [-0.47007019264927452 -44.18471940287516 -10.771837756065759];  % mm
smiData.Solid(1).MoI = [2436.1445209600661 661.46115206189938 2250.8152751107136];  % kg*mm^2
smiData.Solid(1).PoI = [-10.001305215943415 -46.721012441081278 1.1718262952707799];  % kg*mm^2
smiData.Solid(1).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = "khau_xoay_link12.step*:*Default";

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
smiData.Solid(3).mass = 0.077742713439988884;  % kg
smiData.Solid(3).CoM = [93.299052516432894 6.8464808214768968 22.228438023640074];  % mm
smiData.Solid(3).MoI = [23.322048725250749 46.68347910819822 51.818917618723162];  % kg*mm^2
smiData.Solid(3).PoI = [-0.33327452617433839 -0.49205076710550066 0.088131803660706748];  % kg*mm^2
smiData.Solid(3).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = "EndEffector*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(4).mass = 0.08903783961947298;  % kg
smiData.Solid(4).CoM = [86.0631750444511 16.724002889649594 0.10926245254135525];  % mm
smiData.Solid(4).MoI = [84.639502128857302 400.34769694796978 451.69015473284435];  % kg*mm^2
smiData.Solid(4).PoI = [-0.9179228301489184 6.1097750523846193 10.654955278450091];  % kg*mm^2
smiData.Solid(4).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(4).opacity = 1;
smiData.Solid(4).ID = "Link2*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(5).mass = 0.55504291982628284;  % kg
smiData.Solid(5).CoM = [3.3928335602779051e-05 2.6042089298362479 34.482557169530743];  % mm
smiData.Solid(5).MoI = [1486.008152995855 1487.365551195659 1684.2472204081243];  % kg*mm^2
smiData.Solid(5).PoI = [1.1692735437097417 -0.0011220907017473786 0.00099767495396143571];  % kg*mm^2
smiData.Solid(5).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(5).opacity = 1;
smiData.Solid(5).ID = "Base*:*Default";


%============= Joint =============%
%X Revolute Primitive (Rx) %Y Revolute Primitive (Ry) %Z Revolute Primitive (Rz)
%X Prismatic Primitive (Px) %Y Prismatic Primitive (Py) %Z Prismatic Primitive (Pz) %Spherical Primitive (S)
%Constant Velocity Primitive (CV) %Lead Screw Primitive (LS)
%Position Target (Pos)

%Initialize the RevoluteJoint structure array by filling in null values.
smiData.RevoluteJoint(4).Rz.Pos = 0.0;
smiData.RevoluteJoint(4).ID = "";

smiData.RevoluteJoint(1).Rz.Pos = 107.62777322317201;  % deg
smiData.RevoluteJoint(1).ID = "[Link2-1:-:Link3.step-1]";

smiData.RevoluteJoint(2).Rz.Pos = 58.912160976115494;  % deg
smiData.RevoluteJoint(2).ID = "[Link3.step-1:-:EndEffector-1]";

smiData.RevoluteJoint(3).Rz.Pos = -48.728889279145307;  % deg
smiData.RevoluteJoint(3).ID = "[khau_xoay_link12.step-1:-:Link2-1]";

smiData.RevoluteJoint(4).Rz.Pos = 44.736343808679457;  % deg
smiData.RevoluteJoint(4).ID = "[Base-1:-:khau_xoay_link12.step-1]";

