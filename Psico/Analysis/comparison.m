blocks = 4;
trials = 300;
for j=0:blocks-1
  for i=1:trials
    x = block_responsesX(i,j+1) - 960;
    y = block_responsesY(i,j+1) - 540;
    t = atan2(y,x);
    if t < 0
      t = t + 2*pi;
    end
    O(j*trials + i) = mod(block_theta(i, j+1), 2*pi);
    Z(j*trials + i) = t;
    X(j*trials + i) = mod(block_position(i,j+1), 2*pi);
    V(j*trials + i) = block_velocity(i,j+1);
    sigma_z(j*trials + i) = sigma_o(j+1);
  end
end

%Z is the user response, i.e., the measurement with noise
%X is the position with noise, i.e., the process x-state with noise
%V is the velocity with noise, i.e., the process-v state with noise


%%%
%%% KALMAN FILTER
%%%
X_Kalman(1) = 0;
V_Kalman(1) = 0;
P = [1 0; 0 1];
Q = [sigma_x(1) 0; 0 sigma_v(1)];
F = [1 1; 0 1];
H = [1 0] ;
I = [1 0; 0 1];
for i=2:blocks*trials
  R = [sigma_z(i)];
  xe = Xe(i-1) + Ve(i-1);
  ve = Ve(i-1);
  x_hat = [xe; ve];
  
  P = F*P*F' + Q;
  y = [Z(i) - xe];
  S = H*P*H' + R;
  K = P*H'*inv(S);
  x_hat = x_hat + K*y;
  P = (I - K*H)*P;
  
  X_Kalman(i) = x_hat(1);
  V_Kalman(i) = x_hat(2);
end

%%%
%%% DELTA RULE
%%%
X_delta(1) = 0;
alpha = 0.95;
for i=2:blocks*trials
  X_delta(i) = X_delta(i-1) + alpha*(Z(i-1)-X_delta(i-1));
end

%%%
%%% PID
%%%
X_pid(1) = 0;
Kp = 1.0;
Kd = 0;
Ki = 0;
error_int = 0;
last_error = Z(1) - X_delta(1);
for i=2:blocks*trials
  error = Z(i-1) - X_delta(i-1);
  error_int = error_int + error;
  error_dev = error - last_error;
  last_error = error;
  X_pid(i) = X_pid(i-1) + Kp*error + Ki*error_int + Kd*error_dev; 
end