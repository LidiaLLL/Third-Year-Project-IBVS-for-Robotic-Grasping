startup_rvc;

[dots, C, reference_position, Zref, maxDelta, Kp, errorThreshold, maxIterations, dt_control, dt_kin] = initialise();

% Initialize KUKA robot and environment
mdl_KR5;

%Reference config

Start=transl(C)*rpy2tr(0,180,0,'deg');
start_q=KR5.ikine6s(Start);
%SEnd=transl(0.5,0,-0.015)*rpy2tr(0,180,0,'deg');
%end_q=KR5.ikine6s(End);

figure(1);
KR5.plot(start_q);
hold on;
[vertices,faces]=KR5_Cube();
KR5_Table();
hold off;

% Define reference photo coordinates
reference_cam = CentralCamera('focal', 0.01, 'pixel', 10e-6, 'resolution', 1024,'pose', transl(reference_position)*rpy2tr(0,180,0,'deg'));
Reference_Photo = reference_cam.project(dots');
Reference_Photo = Reference_Photo';
%reference_cam.plot(Reference_Photo);

%Define eye-in-hand camera
cam = CentralCamera('focal', 0.01, 'pixel', 10e-6, 'resolution', 1024,'pose', transl(C)*rpy2tr(0,180,0,'deg'));
Current_Photo=cam.project(dots');
Current_Photo = Current_Photo';
%cam.plot(Current_Photo);

plot_photos(Reference_Photo,Current_Photo);


[C] = IBVS_outer_loop(KR5, Reference_Photo, Current_Photo, dots, C, reference_position, Zref, maxDelta, Kp, errorThreshold, maxIterations, dt_control, dt_kin,cam);

Final_Grasp_KR5(C,KR5);

disp('Object Grasped');

function [C] = IBVS_outer_loop(KR5, Reference_Photo, Current_Photo, dots, C, reference_position, Zref, maxDelta, Kp, errorThreshold, maxIterations, dt_control, dt_kin,cam)

    % Define joint limits for the robot
    jointMin = -pi * ones(1,6);
    jointMax = pi * ones(1,6);
    
    desiredOrientation = rpy2tr(0, 180, 0, 'deg');
    resultMatrix = transl(C) * rpy2tr(0,180,0,'deg');
    q = KR5.ikine6s(resultMatrix);

    figure(1);
    hold on;
    KR5.plot(q);
    KR5_Cube();
    KR5_Table();
    hold off;

    lastKinematicUpdate = tic;
    disp('last kinematic update');
    disp(lastKinematicUpdate);
    iteration = 0;

    %Z calculation
    size_ref = norm(Reference_Photo(2, :) - Reference_Photo(1, :));
    size_meas = norm(Current_Photo(2, :) - Current_Photo(1, :));
    scaleFactor = size_meas / size_ref;

    Zmes = Zref * scaleFactor;
    error_im = Reference_Photo - Current_Photo;

    while iteration < maxIterations
        % High-frequency control loop
        
        [controlInput, Current_Photo,error_im] = IBVS_inner_loop(Reference_Photo,Current_Photo,dots, C, reference_position, Zref, Zmes, maxDelta, Kp, errorThreshold, maxIterations, dt_control, dt_kin, lastKinematicUpdate,cam,error_im);
        if check_visibility(Current_Photo, cam) == 0
            disp('Current Photo: ');
            disp(Current_Photo);
            disp('Error: One or more points are out of the camera field of view.');
            return; % Stop the loop
        end
        if all(abs(error_im(:)) < errorThreshold) 
            disp('Finished')
            return;
        end
        
        % Limit the maximum change in coordinates
        controlInput = max(min(controlInput, maxDelta), -maxDelta);
        %controlInput=controlInput';
        disp('Old Coordinates: ');
        disp(C);
        disp('Control Input: ');
        disp(controlInput);
        % Update camera coordinates (C)

        % Integrate the velocity command into the joint position update
        disp(q);
        q = q + controlInput';
        
        % Keep camera/end effector facing vertically downward
        endEffectorPose = KR5.fkine(q);
        C = endEffectorPose.t';
        resultMatrix = transl(C) * rpy2tr(0,180,0,'deg');
        
        % Recompute a refined solution (if necessary) and limit the maximum increment
        q_new = KR5.ikine6s(resultMatrix);
        delta_q = q_new - q;
        delta_q = max(min(delta_q, maxDelta), -maxDelta);
        q = q + delta_q;
        
        % Apply joint limits
        q = max(min(q, jointMax), jointMin);
        
        % Plot updated configuration
        hold on;
        KR5.plot(q);
        hold off;
        disp(q);
        C = endEffectorPose.t';
        disp(C);
    
        % Reset kinematic timer
        lastKinematicUpdate = tic;
    
        disp('IBVS control loop completed.');
        plot_photos(Reference_Photo,Current_Photo);
    end
end


function [controlInput, Current_Photo,error_im] = IBVS_inner_loop(Reference_Photo, Current_Photo,dots, C, reference_position, Zref, Zmes, maxDelta, Kp, errorThreshold, maxIterations, dt_control, dt_kin, lastKinematicUpdate,cam,error_im)
    controlInput = [0;0;0;0;0;0];
    disp(lastKinematicUpdate);
    disp(dt_control);
    while toc(lastKinematicUpdate) < dt_kin
        [Current_Photo,cam] = update_camera(dots,C);
        % Update Current_Photo and check visibility
        disp(Current_Photo)
        if check_visibility(Current_Photo, cam)
            disp('Error: One or more points are out of the camera field of view.');
            break; % Stop the loop
        end

        % Compute the size of the square (assumed from first two points)
        size_ref = norm(Reference_Photo(2,:) - Reference_Photo(1,:));
        size_meas = norm(Current_Photo(2,:) - Current_Photo(1,:));
        scaleFactor = size_meas / size_ref;

        % Estimate current depths by adjusting reference depths using the scale factor
        Zmes = Zref * scaleFactor;

        % Compute image error between reference and measured feature points
        error_im = Reference_Photo - Current_Photo;

        disp(error_im);
        disp(errorThreshold);
        % Optional: if you want to test the visual error frequently
        if all(abs(error_im(:)) < errorThreshold)
            disp('Converged (based on image error)');
            controlInput = [0; 0; 0;0;0;0];  % No control input needed after convergence
            return;
        end

        % Compute and stack the image Jacobian for each measured point
        L_total = [];
        for i = 1:4
            u = Current_Photo(i, 1);
            v = Current_Photo(i, 2);
            % visjac_p computes the partial derivatives of the projection function
            L = cam.visjac_p([u; v], Zmes);
            L=L(:,[1,2,6,10,11,12]);
            L_total = [L_total; L];
        end

        % Compute the control input from the image error and Jacobian.
        % In the original code, only selected components were used.
        controlInput = Kp * (L_total' * error_im(:));
        disp(controlInput);
        %controlInput = controlInput / norm(controlInput);

        % Pause for the control loop period before recalculating control input
        pause(dt_control);
    end
end


%Get Current Image
function [Current_Photo,cam] = update_camera(dots,C)
    cam = CentralCamera('focal', 0.01, 'pixel', 10e-6, 'resolution', 1024, 'pose', transl(C)*rpy2tr(0,180,0,'deg'));
    Current_Photo = cam.project(dots');
    Current_Photo = Current_Photo';
end


function [] = plot_photos(Reference_Photo,Current_Photo)
    %Plot on same graph
    figure(2);
    h1 = scatter(Reference_Photo(:,1),Reference_Photo(:,2));
    hold on;
    h2 = scatter(Current_Photo(:,1),Current_Photo(:,2));
    hold off;
    legend([h1, h2], {'Reference Cooridnates', 'Current Image Coordinates'});
    xlabel('u (pixels)');
    ylabel('v (pixels)');
    set(gca, 'Ydir', 'reverse');
    set(gca, 'YAxisLocation', 'Right');
    xlim([0 1024]);
    ylim([0 1024]);
    grid on;
end

function [dots, C, reference_position, Zref, maxDelta, Kp, errorThreshold, maxIterations, dt_control, dt_kin] = initialise()
    %Robot/Camera Starting Position XYZ Coordinates
    C=[0.8,0.3,0.8];
    %Reference camera psoition
    reference_position=[0.5,0,0.5];
    
    %Dots Position XYZ Coordinates
    dots=[0.53,	0.03, -0.15;
        0.47, 0.03, -0.15;
        0.53, -0.03, -0.15;
        0.47, -0.03, -0.15];    

    Zref = [0.515; 0.515; 0.515; 0.515];
    maxDelta = 0.01;
    Kp = 0.001;
    errorThreshold = 1;
    maxIterations = 1000;
    dt_control = 0.005;  % High-frequency control period
    dt_kin = 0.25;       % Lower-frequency kinematic update period
end

function is_visible = check_visibility(Current_Photo, cam)
    % Camera resolution
    resolution = [1024,1024]; % [width, height], e.g., [1024, 1024]
    width = resolution(1);
    height = resolution(2);
    
    % Check if all points lie within the resolution
    is_visible = all(Current_Photo(:, 1) >= 0 & Current_Photo(:, 1) <= width & ...
                     Current_Photo(:, 2) >= 0 & Current_Photo(:, 2) <= height);
    disp(is_visible);
end

function [] = Final_Grasp_KR5(C,KR5)
    % Number of steps in the animation
    numSteps = 500;
    %If in line with reference photo, must go down in Z direction by 0.515
    D=C+[0,0,-0.515];
    
    figure(1);
    % Animate the robot motion
    for t = linspace(0, 1, numSteps)
        % Interpolate joint angles
        %start_q = start_q + t' * (end_q - start_q);
        C = C + t' * (D - C);
        CT=transl(C)*rpy2tr(0,180,0,'deg');
        q=KR5.ikine6s(CT);
        % Update the robot's configuration
        KR5.plot(q);
        % Pause for a short time to create the animation effect
        pause(0.01);
    end
end