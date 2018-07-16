clear
%camera = webcam;                           % Connect to camera
nnet = alexnet;                            % Load neural net
pic_index = 0;
pic_path = 'C:\Users\tlan\Documents\ownCloud\documents\NBU\学科竞赛\RoboCup\2018\detector\train\';
% while true  
%     %picture = camera.snapshot;             % Take picture
%     pic_index = pic_index + 1;
%     if pic_index > 12
%         pic_index = 1;
%     end
%     pic_path = sprintf('%s%d%s','C:\Users\tlan\Documents\ownCloud\documents\NBU\学科竞赛\RoboCup\2018\detector\test\',pic_index,'.jpg');
%     picture = imread(pic_path);
%     picture = imresize(picture,[227,227]); % Resize
%     label = classify(nnet, picture);       % Classify 
%     image(picture);                        % Show picture
%     title(char(label));                    % Show label
%     drawnow; 
%     pause(1);
% end

filesystem = dir(pic_path);
for i=1:length(filesystem)
    file=filesystem(i);
    name=file.name;
    type=file.isdir;    
    if ~strcmpi(name,'.') && ~strcmpi(name,'..')
        nextpath=fullfile(pic_path,name);
        file.path=nextpath;
        %testname = nextpath
        if type
            filesystem_next = dir(nextpath);
            for j=1:length(filesystem_next)
                file = filesystem_next(j);
                name = file.name;
                type = file.isdir;
                if ~strcmpi(name,'.') && ~strcmpi(name,'..')
                    file_path = fullfile(nextpath,name);
                    file.path=file_path;
                    
                    if ~type
                        testname = file_path;
                        picture = imread(testname);
                        picture = imresize(picture,[227,227]); % Resize
                        label = classify(nnet, picture);       % Classify 
                        image(picture);                        % Show picture
                        title(char(label));                    % Show label
                        drawnow; 
                        pause(1);                        
                    end
                end
            end
        end
    end
end
