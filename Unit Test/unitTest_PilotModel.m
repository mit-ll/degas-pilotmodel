classdef unitTest_PilotModel < matlab.unittest.TestCase
% Copyright 2015 - 2020, MIT Lincoln Laboratory
% SPDX-License-Identifier: X11
%
% UNITTEST_PILOTMODEL Unit test for the UAS Pilot Model block
% using the Matlab unit testing framework
%
% This object uses the Matlab unit testing framework to test the
% constructor, properties and functionality of the UAS Pilot Model block in
% PilotModelUnitTest.slx
    
    methods (TestMethodSetup)
        function setupPathAndBusDefinitions(testCase)
            % Clear the workspace
            evalin('base','clear all');
            warning off;
            
            % Switch to the current directory
            simDir = which('PilotModelUnitTest.slx');
            [simDir,~,~] = fileparts(simDir);
            cd(simDir);
            
            % Populate base workspace with bus_definitions
            bus_definitions();
            avoid_maneuver_bus_definition();
        end
    end
    
    % Test Method Block
    methods (Test)
        % Test Function
        function testConstructor(testCase)
            myPM = HeuristicOperatorModelR6('uasPilot_');
            myDAA = DaidalusV201;
            myDAA.tunableParameterPrefix = 'daaLogic_';
            myMN = MofNFilter('MofN_');
            myHyst = HysteresisFilter('Hyst_');
           
            testCase.assertEqual(myPM.tunableParameterPrefix,'uasPilot_',...
                'PilotModel did not initialize correctly.');
            testCase.assertEqual(myDAA.tunableParameterPrefix,'daaLogic_',...
                'Daidalus did not initialize correctly.');
            testCase.assertEqual(myMN.tunableParameterPrefix,'MofN_',...
                'MofNFilter did not initialize correctly.');
            testCase.assertEqual(myHyst.tunableParameterPrefix,'Hyst_',...
                'HysteresisFilter did not initialize correctly.');
        end
        function testProperties(testCase)
            myPM = HeuristicOperatorModelR6('uasPilot_');
            
            myPM.dt_s = 1;
            myPM.operatorEnabled = true;
            myPM.deterministicMode = false;
            myPM.triggerLevel = 2;
            myPM.alertHoldTime = 4;
            myPM.initialDelayMu = 5;
            myPM.initialDelayMin = 0;
            myPM.coordDelayK = 5.5;
            myPM.coordDelayTheta = 2;
            myPM.coordDelayMin = 0;
            myPM.executionDelayMu = 3;
            myPM.executionDelayMin = 0;
            myPM.probFollowMinDev = 0.8;
            myPM.probLeftTurn = 0.6;
            myPM.probDescend = 0.5;
            myPM.maxRelativeHdg = 40;
            myPM.turnK = 6.21;
            myPM.turnTheta_alt = 8.25; 
            myPM.turnOffset_alt = -30;
            myPM.altitudeK = 9.73;
            myPM.altitudeTheta = 207.98;
            myPM.altitudeOffset = -1500;
            myPM.desiredTurnRate_dps = 3;
            myPM.desiredVertRate_ftpm = 500;
            myPM.reducedTurnAngle_deg = 8;
            myPM.reducedClimbAlt_ft = 50;
            myPM.mentalFilter_numSamp = 1;
            myPM.mentalFilter_climbThresh_fps = 0.5;
            myPM.mentalFilter_turnThresh_rad = 1.2*pi/180;
          
            testCase.assertEqual(myPM.dt_s, 1, "dt_s was not set correctly");
            testCase.assertEqual(myPM.operatorEnabled, true, "operatorEnabled was not set correctly");
            testCase.assertEqual(myPM.deterministicMode, false, "deterministicMode was not set correctly");
            testCase.assertEqual(myPM.triggerLevel, 2, "triggerLevel was not set correctly");
            testCase.assertEqual(myPM.alertHoldTime, 4, "alertHoldTime was not set correctly");
            testCase.assertEqual(myPM.initialDelayMu, 5, "initialDelayMu was not set correctly" );
            testCase.assertEqual(myPM.initialDelayMin, 0, "initialDelayMin was not set correctly");
            testCase.assertEqual(myPM.coordDelayK, 5.5, "coordDelayK was not set correctly");
            testCase.assertEqual(myPM.coordDelayTheta, 2.0, "coordDelayTheta was not set correctly");
            testCase.assertEqual(myPM.coordDelayMin, 0, "coordDelayMin was not set correctly");
            testCase.assertEqual(myPM.executionDelayMu, 3.0, "executionDelayMu was not set correctly");
            testCase.assertEqual(myPM.executionDelayMin, 0, "executionDelayMin was not set correctly");
            testCase.assertEqual(myPM.probFollowMinDev, 0.8, "probFollowMinDev was not set correctly");
            testCase.assertEqual(myPM.probLeftTurn, 0.6, "probLeftTurn was not set correctly");
            testCase.assertEqual(myPM.probDescend, 0.5, "probDescend was not set correctly");
            testCase.assertEqual(myPM.maxRelativeHdg, 40, "maxRelativeHdg was not set correctly");
            testCase.assertEqual(myPM.turnK, 6.21, "turnK was not set correctly");
            testCase.assertEqual(myPM.turnTheta_alt, 8.25, "turnTheta_alt was not set correctly");
            testCase.assertEqual(myPM.turnOffset_alt, -30, "turnOffset_alt was not set correctly");
            testCase.assertEqual(myPM.altitudeK, 9.73, "altitudeK was not set correctly");
            testCase.assertEqual(myPM.altitudeTheta, 207.98, "altitudeTheta was not set correctly");
            testCase.assertEqual(myPM.altitudeOffset, -1500, "altitudeOffset was not set correctly");
            testCase.assertEqual(myPM.desiredTurnRate_dps, 3, "desiredTurnRate_dps was not set correctly");
            testCase.assertEqual(myPM.desiredVertRate_ftpm, 500, "desiredVertRate_ftpm was not set correctly");
            testCase.assertEqual(myPM.reducedTurnAngle_deg, 8, "reducedTurnAngle_deg was not set correctly");
            testCase.assertEqual(myPM.reducedClimbAlt_ft, 50, "reducedClimbAlt_ftdt_s was not set correctly");
            testCase.assertEqual(myPM.mentalFilter_numSamp, 1, "mentalFilter_numSamp was not set correctly");
            testCase.assertEqual(myPM.mentalFilter_climbThresh_fps, 0.5, "mentalFilter_climbThresh_fps was not set correctly");
            testCase.assertEqual(myPM.mentalFilter_turnThresh_rad, 1.2*pi/180, "mentalFilter_turnThresh_rad was not set correctly");
        end
        %Stochastic Mode
        function testStochasticPilotModel(testCase)
            ac2 = BasicAircraftDynamics('ac2dyn_');
            ac2.prepareSim();
           
            PM_Stochastic = HeuristicOperatorModelR6('uasPilot_');
            PM_Stochastic.prepareProperties();
            PM_Stochastic.prepareSim();
           
            DAA = DaidalusV201;
            DAA.tunableParameterPrefix = 'daaLogic_';
            DAA.prepareSim();
           
            MN = MofNFilter('MofN_');
            MN.prepareProperties();
            MN.prepareSim();

            Hyst = HysteresisFilter('Hyst_');        
            Hyst.prepareSim();
           
            %Enable/Disable Maneuvers
            enableVertMan = 0;
            enableHorzMan = 1;
           
            assignin('base', 'enableVertMan', enableVertMan);
            assignin('base', 'enableHorzMan', enableHorzMan);
           
            time = 181; %Can change depending on the encounter
            
            %Alerts and guidance are user-generated for the purpose of this test. 
            
            %The alert level to test the pilot model's response
            alertLevelIn = timeseries(zeros(time,1));
            %Data is assumed to be valid
            isValidFlags = timeseries(ones(time,1));
            %See user manual for daaGuidancedefinition
            daaGuidanceIn = timeseries(zeros(time,283));
            
            %Warning Alert
            alertLevelIn.Data = 3;
            
            %Add altitude-band pairs
            for i = 1:2:11
                daaGuidanceIn.Data(:,271+i) = 1000+250*i;
                daaGuidanceIn.Data(130:150,272+i) = alertLevelIn.Data(end);
            end 
                        
            assignin('base', 'alertLevelIn',alertLevelIn);
            assignin('base', 'isValidFlags',isValidFlags);
            assignin('base', 'daaGuidanceIn',daaGuidanceIn);
            
            %By default the simulation runs in stochastic mode
            [~,~,~] = sim('PilotModelUnitTest.slx');
            
            %Check if the minimum maneuvers (horziontal and vertical) are picked by the pilot model
            
            %Minimum Horizontal Maneuver
            %isMin is generated by chooseMinimumManeuver in the pilot model external functions 
            testCase.assertEqual(double(all(isMin.Data)), 1, "Pilot Model Unit Test Failed. (Aircraft failed to pick minimum maneuver)");
            %Minimum vertical maneuver
            %altFlags is generated by getMinimumAltitudes based on which altitude produces
            %both the minimum altitude change combined with lower alert level
            if(altFlags.Data(:,1) > altFlags.Data(:,2))
                idx = 2*ones(size(altFlags.Data(:,1)));
            else
                idx = ones(size(altFlags.Data(:,1)));
            end
            testCase.assertEqual(idx, minVertMan.Data, "Pilot Model Unit Test Failed. (Aircraft failed to pick minimum maneuver)");
        end
        %Deterministic Mode
        function testDeterministicPilotModel(testCase)
            ac2 = BasicAircraftDynamics('ac2dyn_');
            ac2.prepareSim();
           
            PM_Deterministic = HeuristicOperatorModelR6('uasPilot_');
            PM_Deterministic.deterministicMode = true;
            PM_Deterministic.prepareProperties();
            PM_Deterministic.prepareSim();
           
            DAA = DaidalusV201;
            DAA.tunableParameterPrefix = 'daaLogic_';
            DAA.prepareSim();
           
            MN = MofNFilter('MofN_');
            MN.prepareProperties();
            MN.prepareSim();

            Hyst = HysteresisFilter('Hyst_');        
            Hyst.prepareSim();
           
            %Enable/Disable Maneuvers
            enableVertMan = 0;
            enableHorzMan = 1;
           
            assignin('base', 'enableVertMan', enableVertMan);
            assignin('base', 'enableHorzMan', enableHorzMan);
           
            time = 181; %Can change dependening on the encounter
        
            %The alert level to test the pilot model's response
            alertLevelIn = timeseries(zeros(time,1));
            %Data is assumed to be valid
            isValidFlags = timeseries(ones(time,1));
            %See user manual for daaGuidance definition
            daaGuidanceIn = timeseries(zeros(time,283));
            
            %The folliowing tests whether or not the ownship maneuvers when
            %expected based on suggested guidance and the alert level.
            
            %Alerts and guidance are user-generated for the purpose of this test. 
            
            for i = -1:4
                alertLevelIn.Data = i;
                isValidFlags.Data = 1;
                
                %Add altitude-band pairs
                for j = 1:2:11
                    daaGuidanceIn.Data(:,271+j) = 1000+250*j;
                    daaGuidanceIn.Data(:,272+j) = alertLevelIn.Data(end);
                end

                assignin('base', 'alertLevelIn',alertLevelIn);
                assignin('base', 'isValidFlags',isValidFlags);
                assignin('base', 'daaGuidanceIn',daaGuidanceIn);

                [~,~,~] = sim('PilotModelUnitTest.slx');
                
                %Compute the time before the maneuver (if applicable)
                t = (double(size(Maneuver.Data)) - sum(Maneuver.Data))/10;
                
                if i == -1 || i == 0
                    %No expected maneuver
                    testCase.assertEqual(sum(Maneuver.Data), 0, "Pilot Model Unit Test Failed. (Aircraft maneuvered)");
                elseif i == 1 || i == 2
                    %Check if result matches the sum of the delays (Initial
                    %delay, ATC delay, and execution delay)
                    result = t(1) - (PM_Deterministic.executionDelayMin+PM_Deterministic.coordDelayMin+PM_Deterministic.initialDelayMin);

                    testCase.assertEqual(result, 1, "Pilot Model Unit Test Failed. (Aircraft failed to maneuver when expected)");

                    testCase.assertEqual(double(all(isMin.Data)), 1, "Pilot Model Unit Test Failed. (Aircraft failed to pick minimum horizontal maneuver)");

                    if(altFlags.Data(:,1) > altFlags.Data(:,2))
                        idx = 2*ones(size(altFlags.Data(:,1)));
                    else
                        idx = ones(size(altFlags.Data(:,1)));
                    end
                    testCase.assertEqual(idx, minVertMan.Data, "Pilot Model Unit Test Failed. (Aircraft failed to pick minimum vertical maneuver)");
                else
                    %Check if result matches the sum of the delays (no ATC
                    %delay in these cases)
                    result = t(1) - (PM_Deterministic.executionDelayMin+PM_Deterministic.initialDelayMin);

                    testCase.assertEqual(result, 1, "Pilot Model Unit Test Failed. (Aircraft failed to maneuver when expected)");

                    testCase.assertEqual(double(all(isMin.Data)), 1, "Pilot Model Unit Test Failed. (Aircraft failed to pick minimum horizontal maneuver)");

                    if(altFlags.Data(:,1) > altFlags.Data(:,2))
                        idx = 2*ones(size(altFlags.Data(:,1)));
                    else
                        idx = ones(size(altFlags.Data(:,1)));
                    end
                    testCase.assertEqual(idx, minVertMan.Data, "Pilot Model Unit Test Failed. (Aircraft failed to pick minimum vertical maneuver)");
                end    
            end
            
            %%
            %Preventative alert < 5 seconds (No Maneuver Expected)
            %Clear the array
            alertLevelIn.Data = 0;
            %Four second preventative alert
            alertLevelIn.Data(90:94) = 2;
            assignin('base', 'alertLevelIn',alertLevelIn);
            [~,~,~] = sim('PilotModelUnitTest.slx');
            
            testCase.assertEqual(sum(Maneuver.Data), 0, "Pilot Model Unit Test Failed. (Aircraft maneuvered)");
            
            %%
            %Preventative alert > 5 seconds but < ATC delay (No Maneuver Expected)
            %Clear the array
            alertLevelIn.Data = 0;
            %10 Second Preventative Alert
            alertLevelIn.Data(90:100) = 2;
            assignin('base', 'alertLevelIn',alertLevelIn);
            [~,~,~] = sim('PilotModelUnitTest.slx');
            
            testCase.assertEqual(sum(Maneuver.Data), 0, "Pilot Model Unit Test Failed. (Aircraft maneuvered)");
        end
    end
end