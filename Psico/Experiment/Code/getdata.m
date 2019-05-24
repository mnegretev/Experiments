function [o, dv, x, v] = getdata(n_trial, sigma_o, sigma_x, sigma_v, x0, v0, display)
    
    if nargin < 7
        display = [];
    end
    
    x = nan(n_trial, 1);
    v = nan(n_trial, 1);
    o = nan(n_trial, 1);
    dv = nan(n_trial, 1);
    
    x(1) = x0 + sigma_x * randn(1);
    v(1) = v0 * sign(randn(1)) + sigma_v * randn(1);
    o(1) = x(1) + sigma_o * randn(1);
    dv(1) = sign(v(1));
    
    for i = 2:n_trial
        x(i) = x(i-1) + v(i-1) + sigma_x * randn(1);
        v(i) = v(i-1) + sigma_v * randn(1);
        dv(i) = sign(v(i));
        o(i) = x(i) + sigma_o * randn(1);
    end
    %{
    if ~isempty(display)
        figure; clf;
        subplot(2,1,1);hold on;
        scatter(1:n_trial, x, 'b');
        scatter(1:n_trial, o, 'r');
        xlabel('time');
        ylabel('position');
        legend({'position','observation'});
        subplot(2,1,2);hold on;
        scatter(1:n_trial, v, 'b');
        scatter(1:n_trial, dv, 'r');
        xlabel('time');
        ylabel('velocity');
        legend({'velocity','direction'});
    end
    %}
    
    
end