%sim_Batch.m Script to perform batch quadrotor simulation


Batch = [];
iBatch = 0;
RecAttitudeSucc = [];
RecVAttitTime=[];
RecAttitudeSuccTime= [];
heightLoss=[];
pitchangle=[];
rollangle=[];
v_Impact=[];

for vImpact =2:0.1:3
    for rollImpact = -20:5:-5
        for pitchImpact =-20:5:-10
               yawImpact = 45;
         
            
            iBatch = iBatch + 1;
            disp(iBatch)
   

            [CrashData.ImpactIdentification,CrashData.FuzzyInfo,CrashData.Plot,CrashData.timeImpact] = startsim(vImpact, rollImpact, pitchImpact, yawImpact,iBatch);
            pitchangle(iBatch)=pitchImpact;
            rollangle(iBatch)= rollImpact;
            v_Impact(iBatch)=vImpact;
%             CrashData.roll_atImpact = rollImpact;
%             CrashData.pitch_atImpact = pitchImpact; %in degrees;
%             CrashData.vel_atImpact = vImpact ;
%             CrashData.yaw_atImpact = yawImpact;
            RecVAttitTime(iBatch)=0;
            RecAttitudeSucc(iBatch) = 0;
            RecAttitudeSuccTime(iBatch) = 0;
            heightLoss(iBatch)=0;
            RecVAttitTime(iBatch) = vlookup(CrashData.Plot.times,CrashData.timeImpact); %find the index of the time of impact
            WallNormal=CrashData.ImpactIdentification.wallNormalWorld;
            
            
            for i=RecVAttitTime(iBatch):length(CrashData.Plot.times) %starting from time of impact and on
                     bodyFrameZAxis = quatrotate(CrashData.Plot.quaternions(1:4,i)', [0 0 -1]);
%                        
                             if dot(WallNormal,bodyFrameZAxis) > 0
                                    RecAttitudeSucc(iBatch)=1;
                                    RecAttitudeSuccTime(iBatch)=CrashData.Plot.times(i);
%                                     Distfromwall = sqrt(sum((CrashData.Plot.posns(1:2,i) - CrashData.Plot.posns(1:2, recoveryIdxs(1))).^2));
                                    heightLoss(iBatch) = CrashData.Plot.posns(3, RecVAttitTime(iBatch)) - CrashData.Plot.posns(3, i);
                                    if heightLoss(iBatch) > 1
                                        RecAttitudeSucc(iBatch)=0;
                                    end
%                                     xVelRecovered =  CrashData.Plot.posnDerivs(1,i);
%                                     theta = acos(dot(bodyFrameZAxis, WallNormal));
                                    break
                             end
            end

            
            Batch = [Batch;CrashData];
            
%             
%             if CrashData.Plot.posns(3,end) <=0
%                 if CrashData.Plot.times(end) - CrashData.timeImpact <= 0.9
%                     RecAttitudeSucc(iBatch) = 1;
%                 end
%             end
                    
        end
    end
end

save('Batch.mat')
