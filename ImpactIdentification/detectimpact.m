function [ImpactInfo, ImpactIdentification, timeCalc] = detectimpact(iSim, tStep, ImpactInfo,ImpactIdentification, Histsensors ,Histposes,PREIMPACT_ATT_CALCSTEPFWD, stateDeriv, state)
%detectimpact.m CollisionIdentification phase
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: This phase firstly detects an impact has occured, then
%                estimates the wall normal. Four estimates of the wall
%                normal are calculated (but only 1) is used on Navi
%                experimental platform):
%                   1) Estimate using accelerometer located at IMU_POSN
%                   2) Estimate using accelerometer located at quadrotor CM
%                   3) Estimate 2) corrected with true angular acceleration
%                   4) Estimate 2) corrected with estimated angular
%                   acceleration (consecutive two point differentiation)
%-------------------------------------------------------------------------%
    timeCalc = [0;0;0;0];
    if ImpactInfo.firstImpactDetected == 0     %if impact was never detected go thru this loop    
        Sensor = Histsensors(end); %take last set if sensor data in histogram
        rotMat = quat2rotmat(Histposes(end).attQuat);
        accelWorld = rotMat'*Sensor.accelerometer;
        if norm(accelWorld(1:2))>= 1 && ImpactInfo.firstImpactOccured %Acceleration horizontal magnitude 
            %how if acc world is true anddd firstimpact occured is true? ha
            %impact is 0?
            global IMU_POSN g        
            tic;
            ImpactInfo.firstImpactDetected = 1;
            ImpactIdentification.timeImpactDetected = iSim;            
            
            %pre-impact attitude
            ImpactIdentification.rotMatPreImpact = quat2rotmat(Histposes(end-PREIMPACT_ATT_CALCSTEPFWD).attQuat);      
            
            % 1)
            % wall normal estimate (uncorrected), i.e. e_n, e_N
            % This is what is used on Navi
            % Sensor.accelerometer reading comes from IMU located at
            % IMU_POSN (not at CM)
            rotMat = quat2rotmat(Histposes(end).attQuat);
            ImpactIdentification.inertialAcc = rotMat'*Sensor.accelerometer + [0;0;-1]; %in g's
            wallNormalDirWorld = [ImpactIdentification.inertialAcc(1:2);0];
            ImpactIdentification.wallNormalWorld = wallNormalDirWorld/norm(wallNormalDirWorld);     
            
            % 2)
            % wall normal estimate, i.e. e_n, e_N
            % Sensor.accelerometerAtCM reading comes from IMU located at CM          
            inertialAccCM = rotMat'*Sensor.accelerometerAtCM + [0;0;-1]; %in g's
            wallNormalDirWorld2 = [inertialAccCM(1:2);0];
            ImpactIdentification.wallNormalWorldFromCM = wallNormalDirWorld2/norm(wallNormalDirWorld2);
           
            
            % 3)
            % Corrected wall estimate using true angular acceleration
            inertialAccAtIMU = rotMat'*Sensor.accelerometer + [0;0;-1]; %in g's
            worldAngVel = rotMat'*Sensor.gyro;
            worldAngAcc = rotMat'*stateDeriv(4:6);
            inertialAccAtCMEstimated = inertialAccAtIMU + (cross(worldAngAcc,-rotMat'*IMU_POSN) + cross(worldAngVel,cross(worldAngVel,-rotMat'*IMU_POSN)))/g;
            wallNormalDirWorld3 = [inertialAccAtCMEstimated(1:2);0];
            ImpactIdentification.wallNormalWorldCorrected = wallNormalDirWorld3/norm(wallNormalDirWorld3);
            
            % 4)
            % Corrected wall estimate using estimated angular acceleration
            worldAngAccEstimated = rotMat'*(Sensor.gyro - Histsensors(end-1).gyro)/tStep;
            inertialAccAtCMEstimated2 = inertialAccAtIMU + (cross(worldAngAccEstimated,-rotMat'*IMU_POSN) + cross(worldAngVel,cross(worldAngVel,-rotMat'*IMU_POSN)))/g;
            wallNormalDirWorld4 = [inertialAccAtCMEstimated2(1:2);0];
            ImpactIdentification.wallNormalWorldCorrected2 = wallNormalDirWorld4/norm(wallNormalDirWorld4);
            
            timeCalc(1) = toc;
        end
    end
    
   


end

