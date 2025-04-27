function[] = KR5_Table()
    
    % Create the grey table (a simple flat rectangle)
    tableLength = 2;  % Length of the table
    tableWidth = 2;   % Width of the table
    tableHeight = -0.25;  % Height of the table
    
    % Define the table patch and position it
    table = patch([-tableLength/2, tableLength/2, tableLength/2, -tableLength/2], ...
                  [-tableWidth/2, -tableWidth/2, tableWidth/2, tableWidth/2], ...
                  [0, 0, 0, 0], 'k');  % Black color (default) for the edges
    
    % Set table color and position
    set(table, 'FaceColor', [0.5, 0.5, 0.5]);  % Grey color
    table.ZData = tableHeight * ones(size(table.XData));  % Position the table at z = -0.05

end
