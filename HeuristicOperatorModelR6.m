classdef HeuristicOperatorModelR6 < Block
% Copyright 2015 - 2020, MIT Lincoln Laboratory
% SPDX-License-Identifier: X11
%
%HeuristicOperatorModelR6
%Class wrapper for UAS Pilot Model for Traffic Avoidance Release 6.0. 
%Assumes maneuver guidance from DAIDALUS.

  properties
    
    dt_s = 1
    
    operatorEnabled = true
    deterministicMode = false
    noBufferMode = false;
    
    % alert parameters
    triggerLevel = 2
    alertHoldTime = 4
    
    % timing parameters
    initialDelayMu = 5
    initialDelayMin = 0
    coordDelayK = 5.5
    coordDelayTheta = 2.0
    coordDelayMin = 0
    executionDelayMu = 3.0
    executionDelayMin = 0
    
    %   alert level:   none, prox, prev, corr, warn, recov/WCV
    minUpdateTimes =  [  12,   12,    9,    6,    6,   0]
    meanUpdateTimes = [  12,   12,    6,    3,    3,   3]
    
    seedUpdate
    seedCoordDelay
    seedInitialDelay
    seedExecutionDelay
    
    % maneuver direction parameters
    probFollowMinDev = 0.8
    probLeftTurn = 0.6
    probDescend = 0.5
    maxRelativeHdg = 40
    probTurn = [0.5, 1.0, 1.0; ...      % vert rate <= 0
                0.0, 0.5, 0.65; ...     % vert rate <= 1000 fpm
                0.0, 0.5, 0.4 ];        % vert rate > 1000 fpm
    seedTurn
    seedVertical
    seedChoose
    
    % horizontal maneuver magnitude parameters for minimum suggestion compliant maneuvers
    turnK = 6.21
    turnTheta = 9.67
    turnOffset = -30
    
    % alternate horizontal maneuver magnitude parameters
    turnK_alt = 5.47
    turnTheta_alt = 8.25
    turnOffset_alt = -30
    
    % vertical maneuve magnitude parameters
    altitudeK = 9.73
    altitudeTheta = 207.98
    altitudeOffset = -1500

    % maneuver dynamics parameters
    desiredTurnRate_dps = 3
    desiredVertRate_ftpm = 500
    reducedTurnAngle_deg = 8
    reducedClimbAlt_ft = 50
    
    % mental filter
    mentalFilter_numSamp = 1
    mentalFilter_climbThresh_fps = 0.5
    mentalFilter_turnThresh_rad = 1.2*pi/180;
    
  end
  
  methods % constructor
    function obj = HeuristicOperatorModelR6(tunableParameterPrefix,varargin)
      if( nargin < 1 )
        tunableParameterPrefix = '';
      end
      
      p = inputParser;
      % Required parameters
      addRequired(p,'tunableParameterPrefix',@ischar);

      addOptional(p, 'dt_s', obj.dt_s, @isnumeric);
      addOptional(p, 'operatorEnabled', obj.operatorEnabled, @islogical);
      addOptional(p, 'deterministicMode', obj.deterministicMode, @islogical);
      addOptional(p, 'triggerLevel', obj.triggerLevel, @isnumeric);
      addOptional(p, 'alertHoldTime', obj.alertHoldTime, @isnumeric);
      addOptional(p, 'initialDelayMu', obj.initialDelayMu, @isnumeric);
      addOptional(p, 'initialDelayMin', obj.initialDelayMin, @isnumeric);
      addOptional(p, 'coordDelayK', obj.coordDelayK, @isnumeric);
      addOptional(p, 'coordDelayTheta', obj.coordDelayTheta, @isnumeric);
      addOptional(p, 'coordDelayMin', obj.coordDelayMin, @isnumeric);
      addOptional(p, 'executionDelayMu', obj.executionDelayMu, @isnumeric);
      addOptional(p, 'executionDelayMin', obj.executionDelayMin, @isnumeric);
      addOptional(p, 'minUpdateTimes', obj.minUpdateTimes, @isnumeric);
      addOptional(p, 'meanUpdateTimes', obj.meanUpdateTimes, @isnumeric);
      addOptional(p, 'probFollowMinDev', obj.probFollowMinDev, @isnumeric);
      addOptional(p, 'probLeftTurn', obj.probLeftTurn, @isnumeric);
      addOptional(p, 'probDescend', obj.probDescend, @isnumeric);
      addOptional(p, 'maxRelativeHdg', obj.maxRelativeHdg, @isnumeric);
      addOptional(p, 'probTurn', obj.probTurn, @isnumeric);
      addOptional(p, 'turnK', obj.turnK, @isnumeric);
      addOptional(p, 'turnTheta', obj.turnTheta, @isnumeric);
      addOptional(p, 'turnOffset', obj.turnOffset, @isnumeric);
      addOptional(p, 'altitudeK', obj.altitudeK, @isnumeric);
      addOptional(p, 'altitudeTheta', obj.altitudeTheta, @isnumeric);
      addOptional(p, 'altitudeOffset', obj.altitudeOffset, @isnumeric);
      addOptional(p, 'turnK_alt', obj.turnK, @isnumeric);
      addOptional(p, 'turnTheta_alt', obj.turnTheta_alt, @isnumeric);
      addOptional(p, 'turnOffset_alt', obj.turnOffset_alt, @isnumeric);
      addOptional(p, 'desiredTurnRate_dps', obj.desiredTurnRate_dps, @isnumeric);
      addOptional(p, 'desiredVertRate_ftpm', obj.desiredVertRate_ftpm, @isnumeric);
      addOptional(p, 'reducedTurnAngle_deg', obj.reducedTurnAngle_deg, @isnumeric);
      addOptional(p, 'reducedClimbAlt_ft', obj.reducedClimbAlt_ft, @isnumeric);
      addOptional(p, 'mentalFilter_numSamp', obj.mentalFilter_numSamp, @isnumeric);
      addOptional(p, 'mentalFilter_turnThresh_rad', obj.mentalFilter_turnThresh_rad, @isnumeric);
      addOptional(p, 'mentalFilter_climbThresh_fps', obj.mentalFilter_climbThresh_fps, @isnumeric);
      
      parse(p,tunableParameterPrefix,varargin{:});

      fieldsSet = intersect( fieldnames(p.Results), fieldnames(obj) );
      for i = 1:1:numel(fieldsSet)
        obj.(fieldsSet{i}) = p.Results.(fieldsSet{i});
      end
      
    end
  end % constructor method
  
  methods(Access = 'public')
    function prepareProperties(obj)

      % draw seeds for random error processes

      seeds = randi(4294967295,128); % 4294967295 = 2^32 - 1
      obj.seedCoordDelay = seeds(1);
      obj.seedInitialDelay = seeds(2);
      obj.seedExecutionDelay = seeds(3);
      obj.seedUpdate = seeds(4);
      obj.seedTurn = seeds(5);
      obj.seedVertical = seeds(6);
      obj.seedChoose = seeds(7);
      
      % override probabilistic parameters for deterministic mode
      if obj.deterministicMode
        
        obj.initialDelayMu = 0;
        obj.initialDelayMin = 5;
        obj.coordDelayK = 0;
        obj.coordDelayTheta = 0;
        obj.coordDelayMin = 11;
        obj.executionDelayMu = 0;
        obj.executionDelayMin = 3;
        
        %   alert level:   none, prox, prev, corr, warn, recov/WCV
        obj.minUpdateTimes =  [  12,   12,    9,    6,    6,   3];
        obj.meanUpdateTimes = [  12,   12,    6,    3,    3,   0];

        obj.probFollowMinDev = 1;
        obj.probLeftTurn = 1;
        obj.probDescend = 0;
        obj.probTurn = ones(3,3);

        obj.turnK = 0;
        obj.turnTheta = 0;
        obj.turnOffset = 30;

        obj.turnK_alt = 0;
        obj.turnTheta_alt = 0;
        obj.turnOffset_alt = 15;

        obj.altitudeK = 0;
        obj.altitudeTheta = 0;
        obj.altitudeOffset = 500;
        
        if obj.noBufferMode
            obj.turnOffset = 0;
            obj.turnOffset_alt = 0;
            obj.altitudeOffset = 0;
        end
        
      end
      
    end
  end
  
  methods (Access='protected')
  
        function [ incPaths, srcFiles, libFiles ] = getSourceAndLibs( ~ )
          
          % paths should be relative to CASSATT_HOME
          
          incPaths = {};
          srcFiles = {};
          libFiles = {};
          
        end
  end

  
end % classdef