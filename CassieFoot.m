%Cassie foot force application 

clc
clear all 
close all 

syms c1 c2 r h 


%x represents the travel distance of the linear actuator
function output=calc(x)

h=.032;
r=.055;
c1=.025;
c2=.025;

%x=.02;  %m

% Define the solver function
    function [theta_sol, phi_sol, exitflag] = solveNonlinearEqs(eq1, eq2, initial_guess)
        % SOLVENONLINEAREQS Solves two nonlinear equations dependent on theta and phi
        % using optimization with bounds [0, pi/2].
        %
        % Inputs:
        %   eq1 - Anonymous function of (theta, phi) for the first equation
        %   eq2 - Anonymous function of (theta, phi) for the second equation
        %   initial_guess - [theta0, phi0], initial guess within [0, pi/2]
        %
        % Outputs:
        %   theta_sol - Solution for theta
        %   phi_sol - Solution for phi
        %   exitflag - Optimization exit flag (1 = success, others indicate issues)
        
    
        % Default initial guess if not provided
        if nargin < 3 || isempty(initial_guess)
            initial_guess = [pi/4, pi/4] % Midpoint of [0, pi/2]
        end
    
        % Validate initial guess
        if any(initial_guess < 0) || any(initial_guess > pi/2)
            error('Initial guess must be within [0, pi/2].');
        end
    
        % Define the objective function (sum of squared residuals)
        objective = @(x) (eq1(x(1), x(2))^2 + eq2(x(1), x(2))^2);
    
        % Bounds for theta and phi: [0, pi/2]
        lb = [0, 0];      % Lower bounds
        ub = [pi/2, pi/2]; % Upper bounds
    
        % Optimization options (optional: display iterations)
        options = optimoptions('fmincon', ...
            'Display', 'iter', ... % Show iteration progress
            'Algorithm', 'sqp', ... % Sequential quadratic programming
            'TolFun', 1e-8, ...    % Function tolerance
            'TolX', 1e-8);         % Step size tolerance
    
        % Solve using fmincon
        [x, ~, exitflag] = fmincon(objective, initial_guess, [], [], [], [], ...
                                   lb, ub, [], options);
    
        % Extract solutions
        theta_sol = x(1);
        phi_sol = x(2);
    
        % Verify solution (optional)
        residual1 = eq1(theta_sol, phi_sol);
        residual2 = eq2(theta_sol, phi_sol);
        fprintf('Solution: theta = %.6f, phi = %.6f\n', rad2deg(theta_sol), rad2deg(phi_sol));
        fprintf('Residuals: eq1 = %.2e, eq2 = %.2e\n', residual1, residual2);
        if exitflag <= 0
            warning('Optimization did not converge successfully (exitflag = %d).', exitflag);
        end
    end


%Equation 1 
%angle of tarsal segment relation with theta and phi
psi=@(theta, phi) phi+asin(h/c2*sin(phi));

%Equation 2 
eq1=@(theta, phi) c2/r-sin(theta)*sin(phi)/(sin(pi)-(phi+asin(h/c2*sin(phi))));

%Equaiton 3 
eq2=@(theta, phi) cos(phi)*cos(theta)-(c1-x)/r;

[theta_sol, phi_sol, exitflag]=solveNonlinearEqs(eq1, eq2)

output=[rad2deg(theta_sol) rad2deg(phi_sol)];
end 

x=linspace(0,.053);

thet_vals=[];
phi_vals=[];

vals=[];

for i = 1 : length(x)
    aa=[];
    a= calc(x(i));
    aa=[x(i) (a)];
    vals=[vals transpose(aa)];
end 

thet_vals=vals(2,:);
phi_vals=vals(3,:);

F=1; %% assuming unit force applied by the linear actuators

Fx=[]

for i = 1 : length(x)

    Fx=[Fx F*cos(deg2rad(thet_vals(i)))*cos(deg2rad(phi_vals(i)))*cos(deg2rad(phi_vals(i)))];
end 

plot(x,Fx)
grid on 
xlabel('x travel')
ylabel('Fx')
