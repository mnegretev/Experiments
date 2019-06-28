clear
load("../Data/Raw data/S_1.mat")
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
  xe = X_Kalman(i-1) + V_Kalman(i-1);
  ve = V_Kalman(i-1);
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
error_rms_kalman = 0;
for i=1:1:blocks*trials
  e_kalman(i) = X_Kalman(i) - Z(i);
  error_rms_kalman += sqrt(e_kalman(i)*e_kalman(i));
end
error_rms_kalman /= blocks*trials;
plot(e_kalman)
title("Error using Kalman Filter")
xlabel("Trials")
ylabel("Error [rad]")
legend(["MSE: " mat2str(error_rms_kalman)])



%%%
%%% DELTA RULE
%%%
X_delta(1) = 0;
alpha = 0.95;
for i=2:blocks*trials
  X_delta(i) = X_delta(i-1) + alpha*(Z(i-1)-X_delta(i-1));
end
error_rms_delta = 0;
for i=1:1:blocks*trials
  e_delta(i) = Z(i) - X_delta(i);
  error_rms_delta += sqrt(e_delta(i)*e_delta(i));
end
error_rms_delta /= blocks*trials;
figure
plot(e_delta)
title("Error using Delta Rule")
xlabel("Trials")
ylabel("Error [rad]")
legend(["MSE: " mat2str(error_rms_delta)])


%%%
%%% PID
%%%
X_pid(1) = 0;
Kp = 1.0;
Kd = 0.1;
Ki = 0.01;
error_int = 0;
last_error = Z(1) - X_delta(1);
for i=2:blocks*trials
  error = Z(i-1) - X_delta(i-1);
  error_int = error_int + error;
  error_dev = error - last_error;
  last_error = error;
  X_pid(i) = X_pid(i-1) + Kp*error + Ki*error_int + Kd*error_dev; 
end
error_rms_pid = 0;
for i=1:1:blocks*trials
  e_pid(i) = X_pid(i) - Z(i);
  error_rms_pid += sqrt(e_pid(i)*e_pid(i));
end
error_rms_pid /= blocks*trials;
figure
plot(e_pid)
title("Error using PID Control")
xlabel("Trials")
ylabel("Error [rad]")
legend(["MSE: " mat2str(error_rms_pid)])
