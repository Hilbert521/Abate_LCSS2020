
% Paper:       "Tight Decomposition Functions for Continuous-Time 
%               Mixed-Monotone Systems with Disturbances"
% Publisher:    Control Systems Letters (L-CSS)
% 
% Description: This script generates Figure 1 in the paper. Two tight
%              decomposition functions are constructed for a dynaical 
%              system.  It is shown that the tight decomposition function 
%              proposed in the paper leads to tighter approximations.
%
% Code Author: Matthew Abate
% Date:        6/5/2020

clc; clear all

% Inital set (Given as intervals)
X0 = [-1, 1; ...
      0, 1]

  
% Discretize the intial set (for computation of reachable set)
disc = 20;
holder1 = linspace(X0(1, 1), X0(1, 2), disc);
holder2 = linspace(X0(2, 1), X0(2, 2), disc);
X = [];
for i =1:1:disc
    for j =1:1:disc
        for k =1:1:disc
            X = [X, [holder1(i); holder2(j)]];
        end
    end
end
clear holder1 holder2

k = boundary(X(1, :)', X(2, :)');
X = X(:, k); 
X = X(:);

% Simluate system dynamics in order to compute actual reachable set 
dt = .002;    % time discritization
T = 0: dt :1; % time horizon 
for t = T
    for j = 1:size(X, 1)/2
        xnow = X([2*j - 1, 2*j], 1);
        xnext = xnow + dt* F(xnow);
        X([2*j - 1, 2*j], 1) = xnext;
    end
end
X = reshape(X, 2, size(X, 1)/2); % this variable now contains the boundary 
                                 % of the reachable set


% Set up plot
figure(1); clf; hold on; grid on; 
axis([-1.5 4.5 -2.5 2.5])
xlabel('$x_1$','Interpreter','latex')
xticks([-1.5, 0, 1.5, 3, 4.5])
ylabel('$x_2$','Interpreter','latex')
yticks([-2.5, 0, 2.5])
set(gca,'FontSize',16, 'TickLabelInterpreter','latex')

% Compute approximation of reachble set using alternative decomposition
% function and plot in pink
b = X0(:);
for t = T
    for j = 1:size(X, 1)/2
        b = b + dt*[d2(b(1:2, 1), b(3:4, 1)); d2(b(3:4, 1), b(1:2, 1))];
    end
end
b = reshape(b, 2, 2);
over_approx = [b(1, 1), b(1, 2),b(1, 2),b(1, 1); ...
               b(2, 1), b(2, 1),b(2, 2),b(2, 2)];
patch(over_approx(1, :), over_approx(2, :), 'm', 'FaceAlpha', .2, 'LineWidth', 1.5)

% Compute approximation of reahcable set using tight decomposition function
% and plot in blue
a = X0(:);
for t = T
    for j = 1:size(X, 1)/2
        a = a + dt* [d(a(1:2, 1), a(3:4, 1)); d(a(3:4, 1), a(1:2, 1))];
    end
end
a = reshape(a, 2, 2);
over_approx = [a(1, 1), a(1, 2),a(1, 2),a(1, 1); ...
               a(2, 1), a(2, 1),a(2, 2),a(2, 2)];
patch(over_approx(1, :), over_approx(2, :), 'w', 'LineWidth', 1.5)
patch(over_approx(1, :), over_approx(2, :), 'b', 'FaceAlpha', .2, 'LineWidth', 1.5)

% Plot initial set in red
patch(X(1, :), X(2, :), 'w', 'FaceAlpha', 1, 'LineWidth', 1.5)
b = X0;
over_approx = [b(1, 1), b(1, 2),b(1, 2),b(1, 1); ...
               b(2, 1), b(2, 1),b(2, 2),b(2, 2)];
patch(over_approx(1, :), over_approx(2, :), 'r', 'LineWidth', 1.5)


% Plot actual reachable set in green
patch(X(1, :), X(2, :), 'g', 'FaceAlpha', .8, 'LineWidth', 1.5)

% System Dynamics
function dxdt = F(x)
    dxdt = [abs(x(1) - x(2)); ...
            -x(1)];
end

% Tight Decomopsition Function
function out = d(x, xh)
    if x(2) <= x(1) && x(1) <= xh(2)
        out(1, 1) = 0;
    elseif ( 2*x(1) <= 2*x(2)) && (2*x(1) <= xh(2) + x(2))
        out(1, 1) = x(2)- x(1);
    elseif ( 2*x(1) >= xh(2)) && (2*x(1) >= xh(2) + x(2))
        out(1, 1) = x(1) - xh(2);
    end
    out(2, 1) = - xh(1);
end

% Alternative Decomposition Function
function out = d2(x, xh)
    if x(1) <= xh(1) && xh(1) <= x(2) && x(2) <= xh(2)
        out(1, 1) = x(2) - xh(1);
    elseif x(2) <= xh(2) && xh(2) <= x(1) && x(1) <= xh(1)
        out(1, 1) = x(1)- xh(1);
    elseif x(2) <= xh(1) && x(1) <= xh(2)
        out(1, 1) = 0;
    elseif xh(1) <= min([x(1), xh(2)]) && max([x(1), xh(2)]) <= x(2)
        out(1, 1) = x(2) - xh(1);
    elseif xh(2) <= min([x(2), xh(1)]) && max([x(2), xh(1)]) <= x(2)
        out(1, 1) = x(1) - xh(2);
    elseif xh(1) <= xh(2) && xh(2) <= x(2) && x(2) <= x(1)
        out(1, 1) = x(2) - xh(1);
    elseif xh(2) <= xh(1) && xh(1) <= x(1) && x(1) <= x(2)
        out(1, 1) = x(1)- xh(1);
    end
    out(2, 1) = - xh(1);
end

