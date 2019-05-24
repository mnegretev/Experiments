% This script converts the output of the task into a continuous scale for the
% Analysis. 

clear;
clc;

%Provide direction of Row Data.
data_dir='';
filepath = fullfile(data_dir);
filelist = dir(fullfile(data_dir,'S_*')); 
filelist = {filelist.name}; 
n=length(filelist);

%%
n_blocks=4;

tt=0;
for i=1:n
    clear theta theta1 t    
    filename = filelist{i};
    disp(['processing: ',filename]);
    data = importdata(filename);
    t=0;
    for j=1:n_blocks
        
        t=t+1;

        subjectname=data.subjectname;
        
        %Ground truth
        y = data.block_position(:,j);
        %Observations
        r=data.block_theta(:,j);
        % X coordinates of observations (not necessary given r)
        r1= data.block_observationsX(:,j);
        % Y coordinates of observations (not necessary given r)
        r2= data.block_observationsY(:,j);
        % X coordinates of observations
        R1= data.block_responsesX(:,j);
        % Y coordinates of observations
        R2= data.block_responsesY(:,j);
        %ratioa
        ratio= data.ratio(:,j);
        v = data.block_velocity(:,j);
        centerx=data.length1/2;
        centery=data.height1/2;
        
        %Take the center of the orbit as reference.
        R1=R1-centerx;
        R2=R2-centery;
        r1=r1-centerx;
        r2=r2-centery;
        
        [theta,rho] = cart2pol(R1,R2);
        [theta1,rho1] = cart2pol(r1,r2);
        
        %LAPS
        
        laps=fix(y/(2*pi));
        laps_minus=(fix(y/(2*pi)))-1;
        laps_plus=(fix(y/(2*pi))+1);
        
 %%%%%%%% CONTINUOUS VALUES ACCORDING TO LAPS %%%%%%%%%%%%%%%%
 
     for ix=2:length(y)
 
      theta_real(ix-1)=(laps(ix)*2*pi)+theta(ix-1);
      theta_minus(ix-1)=(laps_minus(ix)*2*pi)+theta(ix-1);
      theta_plus(ix-1)=(laps_plus(ix)*2*pi)+theta(ix-1);
                
     end

      
  %%%%%%%%%%%%%%%%% MINIMIZE ERROR %%%%%%%%%%%%%%%%%%%%%%%%%%%

  theta_real=theta_real';
  theta_minus=theta_minus';
  theta_plus=theta_plus';
  
  %The option with the smallest error is assumed to be the choice made by
  %the participant.
  
  for jx=2:(length(theta_real)+1)
     
      d_real(jx)=abs(y(jx)-theta_real(jx-1));
      d_minus(jx)=abs(y(jx)-theta_minus(jx-1));
      d_plus(jx)=abs(y(jx)-theta_plus(jx-1));
      
      if min([d_real(jx),d_minus(jx),d_plus(jx)])==d_real(jx)
          
         final_responses(jx-1)=theta_real(jx-1);
      
      elseif min([d_real(jx),d_minus(jx),d_plus(jx)])==d_minus(jx)
          
          final_responses(jx-1)=theta_minus(jx-1);
      else
          
          final_responses(jx-1)=theta_plus(jx-1);
          
      end
  end
      
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  % Plots of continuous sequences of responses and observations
  
        %figure(i)
        %subplot(2,2,j)
        %scatter(2:length(final_responses)+1,final_responses);
        %hold on;
        %scatter(1:length(r),r);

        block_theta(:,t)=theta;
        block_theta1(:,t)=theta1;
        block_observationsX(:,t)=r1;
        block_observationsY(:,t)=r2;
        block_responses(:,t)=final_responses;
        block_observations(:,t)=r; 
        block_position(:,t)=y;
        block_velocity(:,t)=v;
        block_ratio(:,t)=ratio;
             
    end
    
    %save(['T_',num2str(i)],'block_ratio','block_responses','block_observations','block_observationsX', 'block_observationsY','block_position','block_velocity','block_vdirection');
    
end

