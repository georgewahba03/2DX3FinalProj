% George Wahba 400371904

clear;
ports = serialportlist("available");
s = serialport(ports(3), 115200);
flush(s);


depth = 10;
measurement_count = 32;
output = zeros(0,3);
i = 0;
% Set the condition for the while loop to continue until all data is received
while (i <= depth*measurement_count-1) % read until all data received

    % Read one line of data from the serial port
    data = readline(s);
    
    % If there is no data to read, exit the loop
    if (isempty(data))
        break;
    end

    % Parse the data and extract the values we want
    tmp = parse(data);
    temp_mat = tmp([2 3 4]);
    
    % Display the extracted values
    disp(temp_mat);
    
    % Add the extracted values to the output matrix
    output = vertcat(output,temp_mat); %#ok<AGROW> 
    
    % Increment the counter for the while loop
    i=i+1;
end

% Transpose the output matrix to get measurement_data
meas_data= output.';

% Convert polar coordinates to Cartesian coordinates
% using the pol2cart function
[x, y, z] = pol2cart(meas_data(2,:).*(pi/180), meas_data(1,:), meas_data(3,:));

% Create a matrix of Cartesian coordinates
cartesian_data = [z; y; x];

% Create a new figure
figure;

% Create a 3D scatter plot of the Cartesian data
scatter3(cartesian_data(1,:), cartesian_data(2,:), cartesian_data(3,:));

% Hold the plot to add more elements to it
hold on;

% Set the initial value of the variable current_depth to 1
current_depth = 1;

% Loop until current_depth reaches the value of depth
while current_depth <= depth
    % Calculate the offset for the current depth
    offset = (current_depth-1)*measurement_count;
    
    % Initialize the counter variable i to 1
    i = 1;
    
    % Loop until i reaches the value of measurement_count-1
    while i <= measurement_count-1
        % Plot a line between two adjacent points in the current ring
        plot3(cartesian_data(1,offset+i:offset+i+1), cartesian_data(2,offset+i:offset+i+1), cartesian_data(3,offset+i:offset+i+1), 'k-');
        % Increment the counter variable i
        i = i + 1;
    end
    
    % Plot a line between the last and first points in the current ring
    plot3([cartesian_data(1,offset+1), cartesian_data(1,offset+measurement_count)], [cartesian_data(2,offset+1), cartesian_data(2,offset+measurement_count)], [cartesian_data(3,offset+1), cartesian_data(3,offset+measurement_count)], 'k-');
    
    % Increment the value of current_depth by 1
    current_depth = current_depth + 1;
end


% Set the current depth to 1 to start the loop
current_depth = 1;

% Loop until the current depth is less than the given depth
while current_depth < depth
    % Set the counter for measurements to 1
    i = 1;
    
    % Loop until all the measurements for the current depth have been plotted
    while i <= measurement_count
        % Calculate the indices of the two points to be connected in the plot
        p1 = (current_depth-1)*measurement_count + i;
        p2 = current_depth*measurement_count - i + measurement_count+1;
        
        % Plot a line connecting the two points in 3D space
        plot3([cartesian_data(1,p1), cartesian_data(1,p2)], [cartesian_data(2,p1), cartesian_data(2,p2)], [cartesian_data(3,p1), cartesian_data(3,p2)], 'k-');
        
        % Increment the counter for measurements
        i = i + 1;
    end
    
    % Increment the current depth counter
    current_depth = current_depth + 1;
end


% Result of scan 
% Clear the hold state of the plot
hold off;

% Set the title and axis labels of the plot
title('COMPENG Final Project Scan');
xlabel('X Depth');
ylabel('Y Width (mm)');
zlabel('Z Height(mm)');

% Turn on the grid lines of the plot
grid on;

% Define a function called 'parse' that takes in a string as input and returns a parsed data array
function parsed_data = parse(n)
    % Display the incoming string for debugging purposes
    incoming_string = n;
    disp("INCOMING STRING: "+ incoming_string);

    % Parse the incoming string and extract the scan variables
    scan_var = sscanf(incoming_string, '%f,%f,%f,%f,%f');
    check_bit = scan_var(1); 
    distance = scan_var(2); 
    angle = scan_var(3)*11.25/16; % Convert angle from raw data to degrees
    depth = scan_var(4);
    spad_num = scan_var(5);
    
    % Return the extracted variables as a parsed data array
    parsed_data = [check_bit, distance angle depth, spad_num];
end
