function [Hbase, CS6] = CS6bot()
theta(1,:) = 0;  d(1,:) = -0.45;   a(1,:) = 0.15;  alpha(1,:) = pi/2;
theta(2,:) = 0;  d(2,:) = 0;       a(2,:) = 0.59;  alpha(2,:) = pi;
theta(3,:) = 0;  d(3,:) = 0;       a(3,:) = 0.13;  alpha(3,:) = -pi/2;
theta(4,:) = 0;  d(4,:) = -0.6471; a(4,:) = 0;     alpha(4,:) = -pi/2;
theta(5,:) = 0;  d(5,:) = 0;       a(5,:) = 0;     alpha(5,:) = pi/2;
theta(6,:) = 0;  d(6,:) = -0.095;  a(6,:) = 0;     alpha(6,:) = pi;

Hbase = [1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1];

theta(2,:) = theta(2,:) + (-pi/2);
theta(3,:) = theta(3,:) + pi/2;
theta(6,:) = theta(6,:) + pi;

CS6 = [theta', d', a', alpha'];
end