[error_over_time_0008, time_array_0008]=Results(0.0008);
[error_over_time_0009, time_array_0009]=Results(0.0009);
[error_over_time_001, time_array_001]=Results(0.001);
[error_over_time_0011, time_array_0011]=Results(0.0011);
[error_over_time_0012, time_array_0012]=Results(0.0012);
[error_over_time_00125, time_array_00125]=Results(0.00125);



figure(1);
plot(time_array_0008, error_over_time_0008, 'LineWidth', 1.5);
hold on;
plot(time_array_0009, error_over_time_0009, 'LineWidth', 1.5);
plot(time_array_001, error_over_time_001, 'LineWidth', 1.5);
plot(time_array_0011, error_over_time_0011, 'LineWidth', 1.5);
plot(time_array_0012, error_over_time_0012, 'LineWidth', 1.5);
plot(time_array_00125, error_over_time_00125, 'LineWidth', 1.5);
%plot(time_array_0013, error_over_time_0013, 'LineWidth', 1.5);
%plot(time_array_0014, error_over_time_0014, 'LineWidth', 1.5);
%plot(time_array_0015, error_over_time_0015, 'LineWidth', 1.5);
%plot(time_array_002, error_over_time_002, 'LineWidth', 1.5);
hold off;
xlabel('Time (s)');
ylabel('Error Magnitude');
title('Error between current and desired features over time');
legend('Kp = 0.0008', 'Kp = 0.0009', 'Kp = 0.0010', 'Kp = 0.0011', 'Kp = 0.0012','Kp = 0.00125','Kp = 0.0013','Kp = 0.0014','Kp = 0.0015');
grid on;



figure(3);
plot(time_array_0008, error_over_time_0008, 'LineWidth', 1.5);
hold on;
plot(time_array_0009, error_over_time_0009, 'LineWidth', 1.5);
plot(time_array_001, error_over_time_001, 'LineWidth', 1.5);
plot(time_array_0011, error_over_time_0011, 'LineWidth', 1.5);
plot(time_array_0012, error_over_time_0012, 'LineWidth', 1.5);
plot(time_array_00125, error_over_time_00125, 'LineWidth', 1.5);
%plot(time_array_0013, error_over_time_0013, 'LineWidth', 1.5);
%plot(time_array_0014, error_over_time_0014, 'LineWidth', 1.5);
%plot(time_array_0015, error_over_time_0015, 'LineWidth', 1.5);
%plot(time_array_002, error_over_time_002, 'LineWidth', 1.5);
hold off;
xlabel('Time (s)');
ylabel('Error Magnitude');
title('Error between current and desired features over time, zoomed in');
legend('Kp = 0.0008', 'Kp = 0.0009', 'Kp = 0.0010', 'Kp = 0.0011', 'Kp = 0.0012','Kp = 0.00125','Kp = 0.0013','Kp = 0.0014','Kp = 0.0015');
grid on;
xlim([20 40]);
ylim([0 400]);




function [error_over_time, time_array]= Results(kp);

    startup_rvc;
    
    [dots, C, reference_position, Zref, maxDelta, Kp, errorThreshold, maxIterations, dt_control, dt_kin, error_over_time, time_array, x_over_time, y_over_time, z_over_time] = initialise(kp);
    
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
    
    [C, error_over_time, x_over_time, y_over_time, z_over_time, time_array, time] = IBVS_outer_loop(KR5, Reference_Photo, Current_Photo, dots, C, reference_position, Zref, maxDelta, Kp, errorThreshold, maxIterations, dt_control, dt_kin,cam,error_over_time, time_array, x_over_time, y_over_time, z_over_time);
    disp('Convergence Time');
    disp(toc(time));
    
    %Final_Grasp_KR5(C,KR5);
    
    %disp('Object Grasped');
    
    %plot end effector trajectory over time
    %figure(3);
    %plot(time_array, error_over_time, 'LineWidth', 1.5);
    %xlabel('Time (s)');
    %ylabel('Error Magnitude');
    %title('Error between current and desired position over time');
    %grid on;
    
    %figure(4);
    %plot(error_over_time, 'LineWidth', 1.5);
    %xlabel('iteration');
    %ylabel('Error Magnitude');
    %title('Error between current and desired position over time');
    %grid on;

end

function [C, error_over_time, x_over_time, y_over_time, z_over_time, time_array, time] = IBVS_outer_loop(KR5, Reference_Photo, Current_Photo, dots, C, reference_position, Zref, maxDelta, Kp, errorThreshold, maxIterations, dt_control, dt_kin,cam, error_over_time, time_array, x_over_time, y_over_time, z_over_time)

    % Define joint limits for the robot
    jointMin = -pi * ones(1,6);
    jointMax = pi * ones(1,6);
    
    %desiredOrientation = rpy2tr(0, 180, 0, 'deg');
    resultMatrix = transl(C) * rpy2tr(0,180,0,'deg');
    q = KR5.ikine6s(resultMatrix);

    figure(1);
    hold on;
    KR5.plot(q);
    KR5_Cube();
    KR5_Table();
    hold off;

    lastKinematicUpdate = tic;
    iteration = 0;

    %Z calculation
    size_ref = norm(Reference_Photo(2, :) - Reference_Photo(1, :));
    size_meas = norm(Current_Photo(2, :) - Current_Photo(1, :));
    scaleFactor = size_meas / size_ref;

    Zmes = Zref * scaleFactor;
    error_im = Reference_Photo - Current_Photo;

    time = tic;

    while iteration < maxIterations
        % High-frequency control loop
        
        
        error_over_time = [error_over_time; sum(abs(error_im(:)))];
        time_array = [time_array; toc(time)];

        [controlInput, Current_Photo, error_im, C] = IBVS_inner_loop(KR5, Reference_Photo,Current_Photo,dots, C, reference_position, Zref, Zmes, maxDelta, Kp, errorThreshold, maxIterations, dt_control, dt_kin, lastKinematicUpdate,cam,error_im, error_over_time, time_array, x_over_time, y_over_time, z_over_time, time);
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

        C = C + controlInput;
        disp('Updated Coordinates: ');
        disp(C);
        iteration = iteration + 1;
        disp(['Iteration ', num2str(iteration), ', Error: ', num2str(sum(abs(error_im(:))))]);

        resultMatrix = transl(C) * rpy2tr(0,180,0,'deg');

        % Recompute a refined solution (if necessary) and limit the maximum increment
        q = KR5.ikine6s(resultMatrix);

        % Plot updated configuration
        figure(1);
        hold on;
        KR5.plot(q);
        hold off;
        disp(q);
        endEffectorPose = KR5.fkine(q);
        C = endEffectorPose.t';
        disp('End effector coordinates: ')
        disp(C);
    
        % Reset kinematic timer
        lastKinematicUpdate = tic;
    
        disp('IBVS control loop completed.');
        plot_photos(Reference_Photo,Current_Photo);
    end
end


function [controlInput, Current_Photo, error_im, C] = IBVS_inner_loop(KR5, Reference_Photo, Current_Photo,dots, C, reference_position, Zref, Zmes, maxDelta, Kp, errorThreshold, maxIterations, dt_control, dt_kin, lastKinematicUpdate,cam,error_im, error_over_time, time_array, x_over_time, y_over_time, z_over_time, time)
    controlInput = [0, 0, 0];
    while toc(lastKinematicUpdate) < dt_kin
        [Current_Photo,cam] = update_camera(dots,C);
        % Update Current_Photo and check visibility

        if check_visibility(Current_Photo, cam) == 0
            disp('Current Photo: ');
            disp(Current_Photo);
            disp('One or more points are out of the camera field of view, robot raising.');
            
            while check_visibility(Current_Photo, cam) == 0
                
                C = C+[0,0,0.01];
                disp('Updated Coordinates: ');
                disp(C);
                
                resultMatrix = transl(C) * rpy2tr(0,180,0,'deg');
        
                % Recompute a refined solution (if necessary) and limit the maximum increment
                q = KR5.ikine6s(resultMatrix);
        
                % Plot updated configuration
                figure(1);
                hold on;
                KR5.plot(q);
                hold off;
                disp(q);
                endEffectorPose = KR5.fkine(q);
                C = endEffectorPose.t';
                disp('End effector coordinates: ')
                disp(C);
                [Current_Photo,cam] = update_camera(dots,C);
                plot_photos(Reference_Photo,Current_Photo)
        
                x_over_time = [x_over_time; C(1)];
                y_over_time = [y_over_time; C(2)];
                z_over_time = [z_over_time; C(3)];
                error_mag = norm(reference_position - C);
                error_over_time = [error_over_time; error_mag];
                time_array = [time_array; toc(time)];
            end
            for i=1:5
                C = C+[0,0,0.01];
                disp('Updated Coordinates: ');
                disp(C);
                
                resultMatrix = transl(C) * rpy2tr(0,180,0,'deg');
        
                % Recompute a refined solution (if necessary) and limit the maximum increment
                q = KR5.ikine6s(resultMatrix);
        
                % Plot updated configuration
                figure(1);
                hold on;
                KR5.plot(q);
                hold off;
                disp(q);
                endEffectorPose = KR5.fkine(q);
                C = endEffectorPose.t';
                disp('End effector coordinates: ')
                disp(C);
                [Current_Photo,cam] = update_camera(dots,C);
                plot_photos(Reference_Photo,Current_Photo)
        
                x_over_time = [x_over_time; C(1)];
                y_over_time = [y_over_time; C(2)];
                z_over_time = [z_over_time; C(3)];
                error_mag = norm(reference_position - C);
                error_over_time = [error_over_time; error_mag];
                time_array = [time_array; toc(time)];
            end
        end
        [Current_Photo,cam] = update_camera(dots,C);

        % Compute image error between reference and measured feature points
        error_im = Reference_Photo - Current_Photo;
        
        disp('Error in coordinates')
        disp(error_im);
        disp('Error Threshold: ');
        disp(errorThreshold);
        % Optional: if you want to test the visual error frequently
        
        if all(abs(error_im(:)) < errorThreshold)
            disp('Converged (based on image error)');
            controlInput = [0, 0, 0];  % No control input needed after convergence
            return;
        end
        
        if all(abs(error_im(1,1:2)) < 1) 
            d1 = Reference_Photo(1,1) - Reference_Photo(2,1);
            d2 = Current_Photo(1,1) - Current_Photo(2,1);
    
            z_error = d1 - d2;

            controlInput = [0,0,Kp*z_error];
        else
            
            x_error = Reference_Photo(1,1) - Current_Photo(1,1);
            y_error = Reference_Photo(1,2) - Current_Photo(1,2);
    
            controlInput = Kp*[x_error,-y_error,0];

        end



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

function [dots, C, reference_position, Zref, maxDelta, Kp, errorThreshold, maxIterations, dt_control, dt_kin, error_over_time, time_array, x_over_time, y_over_time, z_over_time] = initialise(kp)
    %Robot/Camera Starting Position XYZ Coordinates
    C=[0.6,0.3,0.8];
    %Reference camera psoition
    reference_position=[0.5,0,0.5];
    
    %Dots Position XYZ Coordinates
    dots=[0.53,	0.03, -0.15;
        0.47, 0.03, -0.15;
        0.53, -0.03, -0.15;
        0.47, -0.03, -0.15];    

    Zref = [0.515; 0.515; 0.515; 0.515];
    maxDelta = 0.01;
    Kp = kp;
    errorThreshold = 1;
    maxIterations = 50;
    dt_control = 0.005;  % High-frequency control period
    dt_kin = 0.25;       % Lower-frequency kinematic update period
    error_over_time = []; 
    time_array = [];
    x_over_time = [];
    y_over_time = [];
    z_over_time = [];
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
    numSteps = 100;
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
