function data = read_data_3d(filename)
    % Reads the odometry and sensor readings from a file.
    %
    % filename: path to the file to parse
    % data: structure containing the parsed information
    %
    % The data is returned in a structure where the u_t and z_t are stored
    % within a single entry. A z_t can contain observations of multiple
    % landmarks.
    %
    % Usage:
    % - access the readings for timestep i:
    %   data.timestep(i)
    %   this returns a structure containing the odometry reading and all
    %   landmark obsevations, which can be accessed as follows
    % - odometry reading at timestep i:
    %   data.timestep(i).odometry
    % - senor reading at timestep i:
    %   data.timestep(i).sensor
    %
    % Odometry readings have the following fields:
    % - r1 : rotation 1
    % - t  : translation
    % - r2 : rotation 2
    % which correspond to the identically labeled variables in the motion
    % mode.
    %
    % Sensor readings can again be indexed and each of the entris has the
    % following fields:
    % - id      : id of the observed landmark
    % - range   : measured range to the landmark
    % - bearing : measured angle to the landmark (you can ignore this)
    %
    % Examples:
    % - Translational component of the odometry reading at timestep 10
    %   data.timestep(10).odometry.t
    % - Measured range to the second landmark observed at timestep 4
    %   data.timestep(4).sensor(2).range
    input = fopen(filename);

    data = [];
    data_timestep_num=0;
    %data.timestep.sensor = struct;
    first = 1;

    odom = [];
    sensor = [];
    sensor_num = 0;

    while(~feof(input))
        line = fgetl(input);
        arr = strsplit(line, ' ');
        type = deblank(arr{1});

        if(strcmp(type, 'ODOMETRY') == 1)
            if(first == 0)
                data_timestep_num=data_timestep_num+1;
                
                if data_timestep_num==1
                    data.timestep.odometry = odom;
                    data.timestep.sensor = sensor;
                else
                    data.timestep(data_timestep_num).odometry = odom;
                    data.timestep(data_timestep_num).sensor = sensor;
                end
                
                odom = [];
                sensor = [];
                sensor_num = 0;
            end
            first = 0;
            odom.x = str2double(arr{2});
            odom.y  = str2double(arr{3});
            odom.z = str2double(arr{4});
            odom.yaw = str2double(arr{5});
            odom.pitch = str2double(arr{6});
            odom.roll  = str2double(arr{7});
        elseif(strcmp(type, 'SENSOR') == 1)
            sensor_num = sensor_num+1;
            reading = struct;
            reading.id      = str2double(arr{2});
            %reading.range   = str2double(arr{3});
            reading.azim = str2double(arr{3});
            reading.elev = str2double(arr{4});
            
            if sensor_num==1
                sensor = reading;
            else
                sensor(sensor_num) = reading;
            end
        end
    end

    %data.timestep = data.timestep(2:end);

    fclose(input);
end
