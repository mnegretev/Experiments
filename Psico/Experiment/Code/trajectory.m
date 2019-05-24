function [T] = trajectory( centerx,centery,rad )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    an=0.01:0.03:2*pi;
        
    for j=1:length(an)
       theta=an(j);
       xt(j) = centerx + rad*cos(theta);   
       yt(j) = centery + rad*sin(theta);
    end

    T=[xt;yt];
    %scatter(T(1,:),T(2,:));
end

