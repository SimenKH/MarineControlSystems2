clear variables; clc;

% Thruster configuration matrix, extended and non-extended
%not changed yet -Simen
ue_to_t_ = [   0    0 1    0  1     0 1     0;
               1    1 0    1  0     1 0     1;
            39.3 35.3 0 31.3 -5 -28.5 5 -28.5 ];

u_to_t_ = @(a) [   0    0      sin(a(1))                   sin(a(2))                  sin(a(3));
                   1    1      cos(a(1))                   cos(a(2))                  cos(a(3));
                39.3 35.3 31.3*cos(a(1)) -5*sin(a(2))-28.5*cos(a(2)) 5*sin(a(3))-28.5*cos(a(3)) ];

% thrusters weights
%not changed yet -Simen
We_ = diag([1 1 1 1 1 1 1 1]);
W_ = diag([1 1 1 1 1]);

% Constraints
%not changed yet -Simen
thrustLimits = [125; 125; 150; 320; 320]*1000;
throttleLimits = [1.5625; 1.5625; 1.875; 3.2; 3.2]*1000;
angleRateLimits = [0.0209; 0.0209; 0.0209];

% desired force & torque
%not changed yet -Simen
t_des = [15; 60; -20]*1000;

% initial thrust and angle
u_prev = [0; 0; 0];
a_prev = [0; 0; 0];

% some other needed variables
u_indices = [1; 2; 3];
a_indices = [1; 2; 3];

azimuth.mask = [false; false; true; true; true];
azimuth.u_to_ue_index = [1; 2; 3; 5; 7];
azimuth.u_to_a_index = [-1; -1; 1; 2; 3];


%%


% variables that hold which thrusters are and are not considered in the
% current iteration.
u_use = [true; true; true];
a_use = [true; true; true];

u_des = u_prev;
a_des = a_prev;

t_des_subset = t_des;

iter = 0;
feasible = false;
% feasible if all thrust and rate constraints are satisfied
while ~feasible
    
    iter = iter + 1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Assemble configuration matrix
    % (weight matrix W is tbd.)
%     disp(['################## Iteration: ' num2str(iter) ' ##################'])
%     disp(['Using Thrusters subset: ' num2str(u_use') ' | indices: ' num2str(u_indices(u_use)')])
%     disp(['Using angles subset: ' num2str(a_use') ' | indices: ' num2str(a_indices(a_use)')])
%     
    ue_to_t = [];
    
    % iterate through assignable thrusters
    k = 0;
    for i = u_indices(u_use)'    
        % check if current thruster is azimuth or fixed
        
        if azimuth.mask(i) % azimuth thruster
            
            % check if angle is constrained or assignable
            if a_use(azimuth.u_to_a_index(i)) % assignable
                j = azimuth.u_to_ue_index(i);
                ue_to_t = [ue_to_t ue_to_t_(:, j:(j+1))];
                k = k + 2;
            else % constrained
                u_to_t = u_to_t_(a_des);
                ue_to_t = [ue_to_t u_to_t(:, i)];
                k = k + 1;
            end
        else % fixed thruster
            j = azimuth.u_to_ue_index(i);
            ue_to_t = [ue_to_t ue_to_t_(:, j)];
            k = k + 1;
        end
    end
    
    W = eye(k);
    %disp('Configuration matrix:')
    disp(num2str(ue_to_t))
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Calc desired extended thruster force
    
    sz = size(ue_to_t);
    if sz(1) <= sz(2)
        %disp('a');
        t_to_ue = (W \ ue_to_t') / (ue_to_t * (W \ ue_to_t'));
    else
        %disp('b');
        % t_to_ue = (ue_to_t' * (W \ ue_to_t)) \ (ue_to_t' / W);
        t_to_ue = (ue_to_t' * ue_to_t) \ ue_to_t';
    end
    
    ue_des = t_to_ue * t_des_subset;
    
    %disp('Calculated thruster forces subset:');
    %disp(num2str(ue_des'));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % convert to non extended u and angle
    j = 1;
    for i = u_indices(u_use)'
        %disp(['i: ' num2str(i)])
        if azimuth.mask(i) % azimuth thruster
            if a_use(azimuth.u_to_a_index(i)) % assignable
                tmp = sqrt(ue_des(j)^2 + ue_des(j+1)^2);
                u_des(i) = tmp;
                a_des(azimuth.u_to_a_index(i)) = atan2(ue_des(j), ue_des(j+1));
                j = j + 2;
            else
                u_des(i) = ue_des(j);
                j = j + 1;
            end
        else % fixed thruster
            % j = azimuth.u_to_ue_index(i);
            % disp(['f: ' num2str(j)])
            u_des(i) = ue_des(j);
            j = j + 1;
        end
    end
    
    
%     disp('Overall non-extended thruster forces:')
%     disp(['<strong>' num2str(u_des') '</strong>']);
%     disp(['Angles: <strong>' num2str(a_des') '</strong>']);
%     disp('-------------------------------------------')
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % checking thrust rate saturation limits
    u_change = u_des - u_prev;
    u_change_abs = abs(u_change);
    u_change_sign = sign(u_change);
    u_rate_violations = u_change_abs > throttleLimits;
    %disp(['Thrusters that violated rate contraints: ' num2str(u_rate_violations') ' | ' num2str(u_indices(u_rate_violations)')]);
    
    % keep components that do not violate
    u_des_rate_sat = u_prev + u_change .* (~u_rate_violations);
    
    % constrain components that do.
    u_des_rate_sat = u_des_rate_sat + throttleLimits .* u_change_sign .* u_rate_violations;
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % checking magnitude saturation limits
    u_des_rate_sat_mag = abs(u_des_rate_sat);
    u_des_rate_sat_sign = sign(u_des_rate_sat);
    
    u_thrust_violations = u_des_rate_sat_mag > thrustLimits;
    %disp(['Thrusters that violated mag contraints: ' num2str(u_thrust_violations') ' | ' num2str(u_indices(u_thrust_violations)')]);
    
    % keep components that do not violate    
    u_des_sat = u_des_rate_sat .* (~u_thrust_violations);
    
    % constrain components that do.
    u_des_sat = u_des_sat + + thrustLimits .* u_des_rate_sat_sign .* u_thrust_violations;
    %disp(['Saturated thruster forces: <strong>' num2str(u_des_sat') '</strong>']);
    
    u_des = u_des_sat;
    u_violations = u_rate_violations | u_thrust_violations;
    u_use = u_use & (~u_violations);
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % checking angle rate saturation limits
    a_change = a_des - a_prev;
    a_change_abs = abs(a_change);
    a_change_sign = sign(a_change);
    
    a_violations = a_change_abs > angleRateLimits;
    %disp(['Angles that violated rate contraints: ' num2str(a_violations') ' | ' num2str(a_indices(a_violations)')]);
    
    % keep components that do not violate
    a_des_rate_sat = a_prev + a_change .* (~a_violations);
    
    % constrain components that do.
    a_des_rate_sat = a_des_rate_sat + 0.999*angleRateLimits .* a_change_sign .* a_violations;
    %disp(['Saturated thruster angles: <strong>' num2str(a_des_rate_sat') '</strong>']);
    %disp('-------------------------------------------')
    
    a_des = a_des_rate_sat;
    a_use = a_use & (~a_violations);
    
    
    % check if a violating thruster is an azimuth thruster, if so, also
    % contrain the angle
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Calculate the contribution of the constrained thruster forces to the
    % desired overall force.
    
    %disp('Removing the force contribution of the constrained')
    %disp('thrusters from the desired force');
    u_to_t = u_to_t_(a_des);
    disp(num2str(u_to_t(:,u_violations)))
    
    t_con = u_to_t(:,u_violations)*u_des_sat(u_violations);
    %disp(['Contribution: ' num2str(t_con')])
    %disp(['t_des old: ' num2str(t_des_subset')])
    t_des_subset = t_des_subset - t_con;
    %disp(['t_des new: ' num2str(t_des_subset')])
    
    
    
    % if all are constrained => terminate
    % if none violate limits => terminate
    
    % if isempty(find(u_use, 1)) && isempty(find(a_use, 1))
    if isempty(find(u_use, 1))
        % all are constrained
        feasible = true;
       % disp('All thrusters are constrained. Terminating');
    end
    
    if isempty(find(a_violations, true)) && isempty(find(u_violations, true))
        % no contraint/limit violations this iteration
        feasible = true;
       % disp('No limits are violated. Terminating');
    end
    

     if iter > 10
         disp('Something has gone wrong. Terminating');
         feasible = true;
     end


    %feasible = true;
end

% disp('################# Allocation End #################')
% disp('################## ############ ##################')
% disp(['Thrusts: <strong>' num2str(u_des') '</strong>'])
% disp(['Angles: <strong>' num2str(a_des') '</strong>'])
% disp(['Desired Force: <strong>' num2str(t_des') '</strong>'])
% disp(['Actual force: <strong>' num2str((u_to_t_(a_des)*u_des)') '</strong>'])

u_prev = u_des;
a_prev = a_des;




















%%


% why do we need thrust allocation that is more complex than just a 
% pseudoinverse? the thrusters are rate and magnutude constrained anyway, 
% so there is nothing we can do here and åwe should just have a constroller
% that is slow enough so that no rate limits are violated.

% Answer: we can get extra performance out of the system using this smart
% allocation, if there is a redundtant thruster configuration. It is often
% the case that a desired force results in some thrusters to be limited and
% the other not, so one should allocate more force onto the thrusters that
% are not. This is what this algorithm does.