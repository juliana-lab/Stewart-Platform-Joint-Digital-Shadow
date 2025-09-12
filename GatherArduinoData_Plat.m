function [joint_angles] = GatherArduinoData_Plat(inputArg1,inputArg2)
     clear serialObj
    delete(instrfindall);
    delete(instrfind);
    
    serialObj = serialport("COM10", 115200);  
    configureTerminator(serialObj, "LF");
    flush(serialObj);
    
    %% Settings
    collectTime = 22;          
    
    imuID_log = [];
    accel = [];
    gyro  = [];
    mag   = [];
    temp  = [];
    
    startTime = tic;
    
    %% Data Collection
    while toc(startTime) < collectTime
        if serialObj.NumBytesAvailable > 0
            line = readline(serialObj);
            data = str2double(split(line, ','));
    
            if length(data) == 10
                imuID_log(end+1, 1) = data(1);
                accel(end+1, :) = data(2:4);
                gyro(end+1,  :) = data(5:7);
                mag(end+1,   :) = data(8:10);
            else
                fprintf("Skipped (bad format): %s\n", line);
            end
    
        end
    end
    
    try
    % Correct for the Z-flip
    %accel(:,3) = - accel(:,3);
    %accel(:,2) = -accel(:,2);

    catch
        errordlg('An error occurred while processing IMU data. Please check the Arduino connections.', ...
             'Connection Error');
    end


    % U-Joint data holders
    jointBase_0_index = find(imuID_log == 0);
    jointBase_1_index = find(imuID_log == 1);
    jointBase_2_index = find(imuID_log == 2);
    jointBase_3_index = find(imuID_log == 3);
    jointPlat_1_index = find(imuID_log == 4);
    jointPlat_2_index = find(imuID_log == 5);
    jointPlat_3_index = find(imuID_log == 6);
    

    roll = zeros(length(accel),1);
    pitch = zeros(length(accel),1);
    for i=1:length(accel)
        roll(i) = atan2(accel(i,2),accel(i,3))*180/pi;
        pitch(i) = -atan2(-accel(i,1),sqrt(accel(i,2)*accel(i,2)+accel(i,3)*accel(i,3)))*180/pi;
    end
    
    jointBase_0 = [mean(roll(jointBase_0_index)), mean(pitch(jointBase_0_index))];
    jointBase_1 = [mean(roll(jointBase_1_index)), mean(pitch(jointBase_1_index))];
    jointBase_2 = [mean(roll(jointBase_2_index)), mean(pitch(jointBase_2_index))];
    jointBase_3 = [mean(roll(jointBase_3_index)), mean(pitch(jointBase_3_index))];
    jointPlat_1 = [mean(roll(jointPlat_1_index)), mean(pitch(jointPlat_1_index))];
    jointPlat_2 = [mean(roll(jointPlat_2_index)), mean(pitch(jointPlat_2_index))];
    jointPlat_3 = [mean(roll(jointPlat_3_index)), mean(pitch(jointPlat_3_index))];

    joint_angles = [jointPlat_1; jointPlat_2; jointPlat_3];

    %% Save 
    % IMU_Data = struct;
    % IMU_Data.ID     = imuID_log(1:100,:);
    % IMU_Data.Accel  = accel(1:100,:);
    % IMU_Data.Gyro   = gyro(1:100,:);
    % IMU_Data.Mag    = mag(1:100,:);
    % IMU_Data.Temp   = temp(1:100,:);
    % 
    % save('IMU_LoggedData.mat', 'IMU_Data');


end

