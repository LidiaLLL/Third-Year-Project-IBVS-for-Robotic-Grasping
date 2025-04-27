function[vertices,faces] = KR5_Cube()

    tableHeight=-0.25;
    % Create the blue cube
    cubeSize = 0.1;  % Cube side length
    cubePos = [0.45, -0.05, tableHeight];  % Cube position in front of the robot's gripper
    
    % Cube vertices (8 corners of the cube)
    vertices = [
        0, 0, 0;
        cubeSize, 0, 0;
        cubeSize, cubeSize, 0;
        0, cubeSize, 0;
        0, 0, cubeSize;
        cubeSize, 0, cubeSize;
        cubeSize, cubeSize, cubeSize;
        0, cubeSize, cubeSize
    ];
    
    % Cube faces (6 faces of the cube)
    faces = [
        1, 2, 3, 4;   % Bottom face
        5, 6, 7, 8;   % Top face
        1, 2, 6, 5;   % Side face 1
        2, 3, 7, 6;   % Side face 2
        3, 4, 8, 7;   % Side face 3
        4, 1, 5, 8    % Side face 4
    ];
    
    % Translate the cube to the correct position
    vertices = vertices + cubePos;
    
    % Offset distance from face center to place the dots
    offset = 0.03;
    dotRadius = 0.005; % Radius of each dot
    
    % Define the number of points for the circle
    theta = linspace(0, 2*pi, 20);
    circle_x = dotRadius * cos(theta); % X-coordinates for circle
    circle_y = dotRadius * sin(theta); % Y-coordinates for circle
    
    %Plot the cube using patch
    hold on;
    for i = 1:size(faces,1)
        patch('Vertices', vertices, 'Faces', faces(i,:), ...
              'FaceColor', [0 0.7 1], 'EdgeColor', 'k'); % Set face color to blue
    end
    
    faceCenter = mean(vertices(faces(2,:), :));
    
    
    dotPositions = [
        faceCenter + [offset,  offset, 0];
        faceCenter + [-offset, offset, 0];
        faceCenter + [offset, -offset, 0];
        faceCenter + [-offset, -offset, 0];
    ];
    for j = 1:4
        fill3(dotPositions(j,1) + circle_x, dotPositions(j,2) + circle_y, dotPositions(j,3) * ones(size(circle_x)), 'k');
    end
    hold off;
end
