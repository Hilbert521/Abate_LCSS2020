
% Paper:       "Tight Decomposition Functions for Continuous-Time 
%               Mixed-Monotone Systems with Disturbances"
% Publisher:    Control Systems Letters (L-CSS)
% 
% Description: This script generates Figures 2 and 3 in the paper. A tight
%              decomposition functions is constructed for a dynaical 
%              system, and this decomposition function is used to over and
%              underapproxiamte a reachable set.
%
% Code Author: Matthew Abate
% Date:        6/5/2020

clc; clear all;

% Initial set (given as intervals)
X0 = [-1/2 , 1/2; ...
      -1/2 , 1/2; ...
      -1/2,  1/2];

% Disturbance set (given as intervals)
global W
W = [-1/4, 0; ...
      0, 1/4];
   
% check to make sure everything is ordered properly
if X0(1, 2) < X0(1, 1) || X0(2, 2) < X0(2, 1) || ...
    W(2, 2) <  W(2, 1) ||  W(2, 2) <  W(2, 1)
    print('Error: Wrong Ordering')
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% User inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dt = .05;    % Time Discritization  
T  = .5;     % Simulation time 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Setup Plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure 1
figure(1); clf; hold on; grid on; 
axis([-2 2 -2 4 -2 2])
xlabel('$x_1$','Interpreter','latex')
xticks([-6, -4, -2, 0, 2])
ylabel('$x_2$','Interpreter','latex')
yticks([-2, 0, 2, 4])
zlabel('$x_3$','Interpreter','latex')
zticks([-6, -4, -2, 0, 2])
set(gca,'FontSize',16, 'TickLabelInterpreter','latex')
view([.2 .1 .1])

% Plot initial set
holder1 = linspace(X0(1, 1), X0(1, 2), 2);
holder2 = linspace(X0(2, 1), X0(2, 2), 2);
holder3 = linspace(X0(3, 1), X0(3, 2), 2);
Initial_Set = [];
for i =1:2
    for j =1:2
        for k =1:2
            Initial_Set = [Initial_Set; holder1(i), holder2(j), holder3(k)];
        end
    end
end
k_init = boundary(Initial_Set, 0);
trisurf(k_init ,Initial_Set(:, 1), Initial_Set(:, 2), Initial_Set(:, 3),...
    'FaceColor','red','FaceAlpha',1, 'HandleVisibility', 'off')


% figure 2
figure(2); clf; hold on; grid on; 
axis([-3 2 -1 3])
xlabel('$x_1$','Interpreter','latex')
xticks([-3, -2, -1, 0, 1, 2])
ylabel('$x_2$','Interpreter','latex')
yticks([-1, 0, 1, 2, 3])
set(gca,'FontSize',16, 'TickLabelInterpreter','latex')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disc = 5;
holder1 = linspace(X0(1, 1), X0(1, 2), disc);
holder2 = linspace(X0(2, 1), X0(2, 2), disc);
holder3 = linspace(X0(2, 1), X0(2, 2), disc);
Start_Set = [];
for i =1:1:disc
    for j =1:1:disc
        for k =1:1:disc
            Start_Set = [Start_Set, [holder1(i); holder2(j); holder3(k)]];
        end
    end
end
clear holder1 holder2 holder3

disc = 4;
holder1 = linspace(W(1, 1), W(1, 2), disc);
holder2 = linspace(W(2, 1), W(2, 2), disc);
W_Points = [];
for i =1:1:disc
    for j =1:1:disc
        W_Points = [W_Points,  [holder1(i); holder2(j)]];
    end
end
clear holder1 holder2




holder = Start_Set;
holder2 = [];

Embedding_Traj = X0(:);
Embedding_Traj2 = X0(:);

figure(1)
% Simulate system dynamics for reachable set
for t = 0:dt:T
    t
    if t ~= 0
        holder = Next_Set;
    end
    
    for i = 1:size(holder, 2)
        x_now = holder(:, i);
        for j = 1:size(W_Points, 2)
            w_now = W_Points(:, j);
            x_next = x_now + dt*dxdt(x_now, w_now);
            holder2 = [holder2, x_next];
        end
    end
    
    Next_Set = holder2;
    k = boundary(Next_Set', 0.2);
    k = unique(k(:));
    Next_Set = Next_Set(:, k);
    
    holder2 = [];

    % simulate embedding system for over-approximation
    Embedding_Traj = Embedding_Traj + dt*e(Embedding_Traj(1:3), Embedding_Traj(4:6));
    
    % simulate backward time decomposition function for under-approximation
    Embedding_Traj2 = Embedding_Traj2 + dt*E(Embedding_Traj2(1:3), Embedding_Traj2(4:6));
    
    if t ~= 0
        delete(a)
    end

    a(1) = scatter3(Next_Set(1, :), Next_Set(2, :), Next_Set(3, :), ...
                    'b', 'filled', 'HandleVisibility', 'off');
    a(2) = scatter3(Embedding_Traj(1), Embedding_Traj(2), Embedding_Traj(3), 'm','filled', 'HandleVisibility', 'off');
    a(3) = scatter3(Embedding_Traj(4), Embedding_Traj(5), Embedding_Traj(6), 'm','filled', 'HandleVisibility', 'off');
    
    drawnow
end
delete(a)

Next_Set_plot = Next_Set(:, 1:16:end);
k = boundary(Next_Set_plot', 0);
trisurf(k, Next_Set_plot(1, :)', Next_Set_plot(2, :)', Next_Set_plot(3, :)',...
        'FaceColor','green','FaceAlpha', 1, 'EdgeColor', [0, .6, 0], ...
        'HandleVisibility', 'off')

% reshape
Embedding_Traj = [Embedding_Traj(1:3), Embedding_Traj(4:6)];
Embedding_Traj2 = [Embedding_Traj2(1:3), Embedding_Traj2(4:6)];
holder = [];
for i = 1:2
    for j = 1:2
        for k = 1:2
            holder = [holder; [Embedding_Traj(1, i), Embedding_Traj(2, j), Embedding_Traj(3, k)]];
        end
    end
end
k = boundary(holder, 0);
trisurf(k, holder(:, 1), holder(:, 2), holder(:, 3), ...
        'FaceColor','blue','FaceAlpha',.1, 'HandleVisibility', 'off')
drawnow


% compress Figure 1 to x1 x2 plane
figure(2);
a = Embedding_Traj(1:2, :);
over_approx = [a(1, 1), a(1, 2),a(1, 2),a(1, 1); ...
               a(2, 1), a(2, 1),a(2, 2),a(2, 2)];
patch(over_approx(1, :), over_approx(2, :), 'b', 'FaceAlpha', .2)

Initial_Set_2D = [X0(1, 1), X0(1, 2), X0(1, 2), X0(1, 1); ...
                  X0(2, 1), X0(2, 1), X0(2, 2), X0(2, 2)];
patch(Initial_Set_2D(1, :), Initial_Set_2D(2, :), 'r')

Reachable_Set = Next_Set(1:2, :);
k = boundary(Reachable_Set', 0);
Reachable_Set = Reachable_Set(:, k);
patch(Reachable_Set(1, :), Reachable_Set(2, :), 'g')

a = Embedding_Traj2(1:2, :);
under_approx = [a(1, 1), a(1, 2),a(1, 2),a(1, 1); ...
                a(2, 1), a(2, 1),a(2, 2),a(2, 2)];
            
patch(under_approx(1, :), under_approx(2, :), 'w', 'FaceAlpha', 1)
patch(under_approx(1, :), under_approx(2, :), 'm', 'FaceAlpha', .8)
drawnow



% system dynamics
function out = dxdt(x, w)
    out = [ w(1, 1)*x(2, 1)^2 - x(2, 1) + w(2, 1);...
            x(3, 1) + 3; ...
            x(1, 1) - x(2, 1) - w(1, 1)^3];
end

% embedding function for d
function out = e(x, xh)
    global W
    out = [d(x,  W(:, 1), xh, W(:, 2)); ...
           d(xh, W(:, 2), x,  W(:, 1))];
end

% embedding function for D
function out = E(x, xh)
    global W
    out = [D(x,  W(:, 1), xh, W(:, 2)); ...
           D(xh, W(:, 2), x,  W(:, 1))];
end

% backward time decomposition function
function out = D(x, w, xh, wh)
    out = d(xh, wh, x, w);
end

% forward time decomposition function
function out = d(x, w, xh, wh)
    points = [x(2); ...
              xh(2)];
          
    if sum(x <= xh) == 3
        xu = x;
        xo = xh;
        mode = 'under_approximation';
    else
        xu = xh;
        xo = x;
        mode = 'over_approximation';
    end
       
    if xu(2) <= 0 && 0 <= xo(2)
        points(end + 1, 1) = 0;
    end
    if xu(2) <= 1/(2*w(1)) && 1/(2*w(1)) <= xo(2)
        points(end + 1, 1) = 1/(2*w(1));
    end
    
    holder = zeros(size(points, 1), 1);
    for i=1:size(points, 1)
        a = dxdt([0; points(i, :); 0], w);
        holder(i, 1) = a(1);
    end
    
    if isequal(mode, 'under_approximation')
        d1 = min(holder);
    elseif isequal(mode, 'over_approximation')
        d1 = max(holder);
    else
        error('')
    end
    
    d2 = x(3, 1) + 3;
    d3 = x(1, 1) - xh(2, 1) - wh(1, 1)^3;
    
    out = [d1; d2; d3];
end






