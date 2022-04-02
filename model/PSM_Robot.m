function msfuntmpl_basic(block)
%MSFUNTMPL_BASIC A Template for a Level-2 MATLAB S-Function
%   The MATLAB S-function is written as a MATLAB function with the
%   same name as the S-function. Replace 'msfuntmpl_basic' with the 
%   name of your S-function.

%   Copyright 2003-2018 The MathWorks, Inc.

%%
%% The setup method is used to set up the basic attributes of the
%% S-function such as ports, parameters, etc. Do not add any other
%% calls to the main body of the function.
%%
setup(block);

%endfunction

%% Function: setup ===================================================
%% Abstract:
%%   Set up the basic characteristics of the S-function block such as:
%%   - Input ports
%%   - Output ports
%%   - Dialog parameters
%%   - Options
%%
%%   Required         : Yes
%%   C MEX counterpart: mdlInitializeSizes
%%
function setup(block)

% Register number of ports
block.NumInputPorts  = 1;
block.NumOutputPorts = 2;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
block.InputPort(1).Dimensions        = 6;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = false;

% Override output port properties
block.OutputPort(1).Dimensions       = 6;
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';
block.OutputPort(1).SamplingMode = 'Sample';

block.OutputPort(2).Dimensions       = 6;
block.OutputPort(2).DatatypeID  = 0; % double
block.OutputPort(2).Complexity  = 'Real';
block.OutputPort(2).SamplingMode = 'Sample';

% Register parameters
block.NumDialogPrms     = 3; % initial states q dq; gravity

block.NumContStates = 12;



% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [-1 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';

%% -----------------------------------------------------------------
%% The MATLAB S-function uses an internal registry for all
%% block methods. You should register all relevant methods
%% (optional and required) as illustrated below. You may choose
%% any suitable name for the methods and implement these methods
%% as local functions within the same file. See comments
%% provided for each function for more information.
%% -----------------------------------------------------------------

% block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
block.RegBlockMethod('InitializeConditions', @InitializeConditions);
% block.RegBlockMethod('Start', @Start);
block.RegBlockMethod('Outputs', @Outputs);     % Required
% block.RegBlockMethod('Update', @Update);
block.RegBlockMethod('Derivatives', @Derivatives);
  block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);
% block.RegBlockMethod('Terminate', @Terminate); % Required

%end setup

function SetInpPortFrameData(block, idx, fd)
  
  block.InputPort(idx).SamplingMode = fd;
  block.OutputPort(1).SamplingMode  = fd;
  
%endfunction

%%
%% PostPropagationSetup:
%%   Functionality    : Setup work areas and state variables. Can
%%                      also register run-time methods here
%%   Required         : No
%%   C MEX counterpart: mdlSetWorkWidths
%%
% function DoPostPropSetup(block)

% block.NumDworks = 1;
%   
%   block.Dwork(1).Name            = 'x1';
%   block.Dwork(1).Dimensions      = 1;
%   block.Dwork(1).DatatypeID      = 0;      % double
%   block.Dwork(1).Complexity      = 'Real'; % real
%   block.Dwork(1).UsedAsDiscState = true;


%%
%% InitializeConditions:
%%   Functionality    : Called at the start of simulation and if it is 
%%                      present in an enabled subsystem configured to reset 
%%                      states, it will be called when the enabled subsystem
%%                      restarts execution to reset the states.
%%   Required         : No
%%   C MEX counterpart: mdlInitializeConditions
%%
function InitializeConditions(block)
% block.ContStates.Data(1).Dimensions = 6;
% block.ContStates.Data(1).DatatypeID      = 0;      % double
% block.ContStates.Data(1).Complexity      = 'Real'; % real
% 
% block.ContStates.Data(2).Dimensions = 6;

block.ContStates.Data(1) = block.DialogPrm(1).Data(1);
block.ContStates.Data(1) = block.DialogPrm(1).Data(2);
block.ContStates.Data(1) = block.DialogPrm(1).Data(3);
block.ContStates.Data(1) = block.DialogPrm(1).Data(4);
block.ContStates.Data(1) = block.DialogPrm(1).Data(5);
block.ContStates.Data(1) = block.DialogPrm(1).Data(6);

block.ContStates.Data(2) = block.DialogPrm(2).Data(1);
block.ContStates.Data(2) = block.DialogPrm(2).Data(2);
block.ContStates.Data(2) = block.DialogPrm(2).Data(3);
block.ContStates.Data(2) = block.DialogPrm(2).Data(4);
block.ContStates.Data(2) = block.DialogPrm(2).Data(5);
block.ContStates.Data(2) = block.DialogPrm(2).Data(6);

%end InitializeConditions


%%
%% Start:
%%   Functionality    : Called once at start of model execution. If you
%%                      have states that should be initialized once, this 
%%                      is the place to do it.
%%   Required         : No
%%   C MEX counterpart: mdlStart
%%
% function Start(block)
% 
% block.Dwork(1).Data = 0;

%end Start

%%
%% Outputs:
%%   Functionality    : Called to generate block outputs in
%%                      simulation step
%%   Required         : Yes
%%   C MEX counterpart: mdlOutputs
%%
function Outputs(block)

  block.OutputPort(1).Data = [block.ContStates.Data(1),block.ContStates.Data(2),block.ContStates.Data(2),block.ContStates.Data(4),block.ContStates.Data(5),block.ContStates.Data(6)];
  block.OutputPort(2).Data = [block.ContStates.Data(7),block.ContStates.Data(8),block.ContStates.Data(9),block.ContStates.Data(10),block.ContStates.Data(11),block.ContStates.Data(12)];

%end Outputs

%%
%% Update:
%%   Functionality    : Called to update discrete states
%%                      during simulation step
%%   Required         : No
%%   C MEX counterpart: mdlUpdate
%%
% function Update(block)
% 
% block.Dwork(1).Data = block.InputPort(1).Data;

%end Update

%%
%% Derivatives:
%%   Functionality    : Called to update derivatives of
%%                      continuous states during simulation step
%%   Required         : No
%%   C MEX counterpart: mdlDerivatives
%%
function Derivatives(block)

g = block.DialogPrm(2).Data;
u =  block.InputPort(1).Data;

q = [block.ContStates.Data(1),block.ContStates.Data(2),block.ContStates.Data(2),block.ContStates.Data(4),block.ContStates.Data(5),block.ContStates.Data(6)];
dq = [block.ContStates.Data(7),block.ContStates.Data(8),block.ContStates.Data(9),block.ContStates.Data(10),block.ContStates.Data(11),block.ContStates.Data(12)];
g_num = g;


M = M_fun(q);
C = C_fun(q,dq);
G = G_fun(q,g_num);

ddq = M\(u - C - G);

% dq_ddq = [dq;ddq];

block.Derivatives.Data(1) = dq(1);
block.Derivatives.Data(2) = dq(2);
block.Derivatives.Data(3) = dq(3);
block.Derivatives.Data(4) = dq(4);
block.Derivatives.Data(5) = dq(5);
block.Derivatives.Data(6) = dq(6);

block.Derivatives.Data(7) = ddq(1);
block.Derivatives.Data(8) = ddq(2);
block.Derivatives.Data(9) = ddq(3);
block.Derivatives.Data(10) = ddq(4);
block.Derivatives.Data(11) = ddq(5);
block.Derivatives.Data(12) = ddq(6);



%end Derivatives

%%
%% Terminate:
%%   Functionality    : Called at the end of simulation for cleanup
%%   Required         : Yes
%%   C MEX counterpart: mdlTerminate
%%
% function Terminate(block)

%end Terminate

