%function [] = usart_jpg(file)
%img=imread('C:\Users\tlan\Pictures\19C228DD909F105103870D462716382E.jpg');
img=[];
serial_object = serial('COM5','BaudRate',115200);

fopen(serial_object);
rec_bytes=1;
rec_end = 0;
while (rec_end == 0)
    str_len = 0;
    if(serial_object.BytesAvailable > 1)
        str_len = serial_object.BytesAvailable;
        idn = fread(serial_object,str_len);
        %str = sprintf('received length = %d',str_len)
    end
    
    for i=1:str_len
        rec_bytes = rec_bytes + 1;
        img(rec_bytes) = idn(i);
        
        
        if(idn(i)==255 && idn(i+1) == 216)
            str = sprintf('0x%x 0x%x',idn(i),idn(i+1))
            img(1)= 255;
            %img(2) = 216;
            rec_bytes = 1;
            
        end
        if(idn(i)==217 && idn(i-1) == 255)
            str = sprintf('0x%x 0x%x',idn(i-1),idn(i))
        %rec_bytes = rec_bytes + 1;
        %img(rec_bytes) = idn(i);
            rec_end = 1;
        end
    end
    %str = sprintf('0x%x',str_len)

    %disp(str_len);

    pause(0.001);
end

%fclose(data_file);
fclose(serial_object);
clear serial_object;

%imshow(img)
data_file = fopen('usart_jpg.jpg','w');
fwrite(data_file,img);
size(img)
%return
