function [] = tiny_stream(file)
% TINY_STREAM Plot and record a live pulse oximeter data stream.
%   [] = TINY_STREAM(file) plots a live pulse oximeter data stream, and also
%   records it to file. The data stream must contain ten columns of data.
%串口传过来的数据个数，以回车换行结束。譬如'123  342'代表有两个数据，以空格分开
expected_column_count = 3;

%% Declare variables.
% The sample rate is currently set to 50Hz by the pulse oximeter firmware, and
% only the last 10 cycles are plotted. Declaring plot data beforehand makes it
% easier to configure the plot.
global MAIN_LOOP;
MAIN_LOOP = true;

sample_rate = 50;
cycle_count = 10;

red_lowpass_data = NaN(sample_rate * cycle_count,1);

%% Create and configure plot.
% The 'plot' command is not used to update the plot, as it is far too slow to
% handle data streaming at 50Hz. Instead, the existing plot data is updated.
close all;
strip_chart = figure('Renderer','painters');
set(strip_chart,'DoubleBuffer','on');
set(strip_chart,'KeyPressFcn',@stop_stream);

time = 1:(sample_rate * cycle_count);

% Configure red chart.
%red_subplot = subplot(2,1,1);
red_subplot = subplot(1,1,1);
red_lowpass_trace = plot(time,red_lowpass_data,'b');

% set(red_subplot,'YTickLabel',...
%     {'-10,000','0','10,000','20,000','30,000','40,000'},...
%     'YTick',[-10000 0 10000 20000 30000 40000],...
%     'YGrid','on');
% ylim(red_subplot,[-10000 40000]);
%下面更改Y轴的显示的范围以及坐标标示
set(red_subplot,'YTickLabel',...
    {'-1000','-800','-600','-400','-200','0','200','400','600','800','1000'},...
    'YTick',[-1000 -800 -600 -400 -200 0 200 400 600 800 1000],...
    'YGrid','on');
ylim(red_subplot,[-1000 1000]);
ylabel('count');

set(red_subplot,'XTickLabel',[],...
    'XTick',(0:sample_rate:(sample_rate * cycle_count))',...
    'XGrid','on');
xlim(red_subplot,[0 sample_rate * cycle_count]);
xlabel('1 sec / div');

%注释掉下面这句，图就充满整个窗口了
%set(red_subplot,'OuterPosition',[0 0.5 1 0.5]);
box(red_subplot,'on');
%title('Red Data');
title('关闭窗口前，在本窗口中按任意键结束读取程序，正确关闭串口！！！');
set(red_subplot,'ALimMode','manual',...
    'CameraPositionMode','manual',...
    'CameraTargetMode','manual',...
    'CameraUpVectorMode','manual',...
    'CLimMode','manual',...
    'TickDirMode','manual',...
    'XLimMode','manual','YLimMode','manual','ZLimMode','manual',...
    'XTickMode','manual','YTickMode','manual','ZTickMode','manual',...
    'XTickLabelMode','manual','YTickLabelMode','manual',...
    'ZTickLabelMode','manual');

drawnow;

%% Create and configure the serial port object.
serial_object = serial('COM5');

serial_object.BaudRate = 115200;
serial_object.DataBits = 8;
serial_object.FlowControl = 'none';
serial_object.StopBits = 1;

%% Configure the data stream.
% Note the use of the callback function 'transfer_data'. This is called by
% MATLAB whenever it detects the specified terminator in the serial object
% buffer.
serial_object.Terminator = 'CR/LF';
serial_object.InputBufferSize = 2^18;
serial_object.BytesAvailableFcnMode = 'terminator';
serial_object.BytesAvailableFcn = {@transfer_data};

serial_object.UserData.string_data = [];
serial_object.UserData.is_new = false;

%% Open the serial port.
if strcmp(serial_object.Status,'closed')
    fopen(serial_object);
end

%% Open the file.
%注释掉，不保存数据了
%data_file = fopen(file,'w');

%% Main program loop.
% There may be more than one row of source data in the serial input buffer at
% the start of the main program loop. Any of these rows may be incomplete, so
% the first thing the main program does is to check that the data contains the
% expected number of entries. If it does not, then the entire data chunk is
% discarded.
while MAIN_LOOP == true
    if serial_object.UserData.is_new == true
        chunk_string = serial_object.UserData.string_data;
        serial_object.UserData.is_new = false;

        chunk_numeric = sscanf(chunk_string,'%d');
        %disp(chunk_numeric);
        chunk_length = length(chunk_numeric);

        if mod(chunk_length, expected_column_count) == 0
            data_column_count = chunk_length / expected_column_count;

            data = reshape(chunk_numeric,expected_column_count,...
                data_column_count);

            %保存到数据文件中，暂时不用，注释掉了
            %fprintf(data_file,...
            %    '%6d %6d %6d %6d %6d %6d %6d %6d %6d %6d\r\n',data);

            % Update red subplot.
            red_lowpass_data = get(red_lowpass_trace,'YData');

            red_lowpass_data(1,1:end - data_column_count) =...
                red_lowpass_data(1,data_column_count + 1:end);
            
            %下面的data(1,:)表示显示第1个数据，如果要显示第2个数据，改成data(2,:)
            red_lowpass_data(1,end - data_column_count + 1:end) =...
                data(1,:);

            set(red_lowpass_trace,'YData',red_lowpass_data);

            drawnow;
        end
    end

    pause(0.001);
end

%fclose(data_file);
fclose(serial_object);
delete(serial_object);
clear serial_object;

return

%% Loop control.
function [] = stop_stream(source, event)
% STOP_STREAM Stop the pulse oximeter serial stream.
%   STOP_STREAM(source, event) sets the MAIN_LOOP global variable to false
%   whenever a key is pressed while plot has focus.
global MAIN_LOOP;

MAIN_LOOP = false;

return

%% Data transfer.
function [] = transfer_data(object, event)
% TRANSFER_DATA Transfer data between buffers.
%   TRANSFER_DATA(object, event) transfers data between the serial object
%   input buffer and the user data area of the serial object.
string_data = fgets(object);

if object.UserData.is_new == false
    object.UserData.string_data = string_data;
    object.UserData.is_new = true;
else
    object.UserData.string_data = [object.UserData.string_data string_data];
end

return