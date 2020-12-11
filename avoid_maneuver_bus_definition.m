function avoid_maneuver_bus_definition() 
% Copyright 2015 - 2020, MIT Lincoln Laboratory
% SPDX-License-Identifier: X11
%
% AVOID_MANEUVER_BUS_DEFINITION initializes a bus object for avoid maneuver
% choices in the MATLAB base workspace

clear elems;

% timestamp
elems(1) = Simulink.BusElement;
elems(1).Name = 'time';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];

% maneuver type (1 = horizontal, 2 = vertical)
elems(2) = Simulink.BusElement;
elems(2).Name = 'type';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];

% Desired altitude (ft) or heading (deg)
elems(3) = Simulink.BusElement;
elems(3).Name = 'desired_state';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];

% Reference altitude or heading
elems(4) = Simulink.BusElement;
elems(4).Name = 'reference_state';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];

% WC success flag (1 indicates maneuver is expected to avoid loss of WC)
elems(5) = Simulink.BusElement;
elems(5).Name = 'wc_flag';
elems(5).Dimensions = 1;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'double';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).SamplingMode = 'Sample based';
elems(5).Min = [];
elems(5).Max = [];

AvoidanceManeuver = Simulink.Bus;
AvoidanceManeuver.HeaderFile = '';
AvoidanceManeuver.Description = sprintf('Self-separation maneuver selected by the UAS operator model.');
AvoidanceManeuver.DataScope = 'Auto';
AvoidanceManeuver.Alignment = -1;
AvoidanceManeuver.Elements = elems;
assigninContext('AvoidanceManeuver', AvoidanceManeuver)