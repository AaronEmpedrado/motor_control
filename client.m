%Modified the code to take out 'flowcontrol' and 'hardware'
%also added the "x" option
function client(port)
%   provides a menu for accessing PIC32 motor control functions
%
%   client(port)
%
%   Input Arguments:
%       port - the name of the com port.  This should be the same as what
%               you use in screen or putty in quotes ' '
%
%   Example:
%       client('/dev/ttyUSB0') (Linux/Mac)
%       client('COM3') (PC)
%
%   For convenience, you may want to change this so that the port is hardcoded.

%Package needed to use dictionary mapping
import containers.*

% Opening COM connection
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

%Hardcoded port -> change if necessary
port = '/dev/tty.usbserial-DM01N1JX';

fprintf('Opening port %s....\n',port);

% settings for opening the serial port. baud rate 230400, hardware flow control
% wait up to 120 seconds for data before timing out
mySerial = serial(port, 'BaudRate', 230400, 'Timeout',120); 
% opens serial connection
fopen(mySerial);
% closes serial port when function exits
clean = onCleanup(@()fclose(mySerial));                                 

has_quit = false;

%Dictionary mapping to help with displaying modes
MODES = containers.Map({0,1,2,3,4}, {'IDLE','PWM','ITEST','HOLD','TRACK'});

% menu loop
while ~has_quit
    fprintf('PIC32 MOTOR DRIVER INTERFACE\n\n');
    % display the menu options; this list will grow
    fprintf(['\ta: Read current sensor (ADC counts)\tb: Read current sensor(mA)\n'...
                    '\tc: Read encoder(counts)\t\t\td: Read encoder (deg)\n '...
                   '\te: Reset encoder\t\t\tf: Set PWM(-100 to 100)\n'...
                   '\tg: Set current gains\t\t\th: Get current gains\n'...
                   '\ti: Set position gains\t\t\tj: Get position gains\n'...
                   '\tk: Test current control\t\t\tl: Go to angle(deg)\n'...
                   '\tm: Load step trajectory\t\t\tn: Load cubic trajectory\n'...
                   '\to: Execute trajectory\t\t\tp: Unpower the motor\n'...
                   '\tq: Unpower the motor\t\t\tq: Quit the client\n'...
                   '\tr: Get mode\n']);    % read the user's choice
    selection = input('\nENTER COMMAND: ', 's');
     
    % send the command to the PIC32
    fprintf(mySerial,'%c\n',selection);
    
    % take the appropriate action
    switch selection
        %Read encoder counts
        case 'c'                         
            counts = fscanf(mySerial, '%d');    %Read the count
            fprintf('The motor angle is %d counts.\n', counts);
        %Read encoder in degrees 
        case 'd'                         
            degrees = fscanf(mySerial, '%d');   %read the count (deg)
            fprintf('The motor angle is %d degrees.\n', degrees);
        %Resets the encoder to 32,768
        case 'e'                         
            fprintf('Encoder has been reset.\n');
        %Reads the Mode
        case 'r'
            mode = fscanf(mySerial, '%d');  %read the numeric value of the mode
            fprintf('PIC32 currently set to: %s\n', MODES(mode));     %display the current mode
        %Reads the current sensor in terms of ADC counts
        case 'a'
            adc = fscanf(mySerial, '%d');    %read the adc counts from pic
            fprintf('Current sensor reads: %d ADC counts\n', adc);
        %Reads the current sensor in terms of mA
        case 'b'
            current = fscanf(mySerial, '%d');
            fprintf('Current sensor reads: %d mA\n', current);
        %Sets the PWM
        case 'f'
            pwm = input('Enter PWM (-100 to 100): ');      %Get input
            fprintf(mySerial,'%d\n', pwm);                      %Send to the PIC
        %Unpower the motor
        case 'p'
            fprintf('Motor has been unpowered.\n');
        %Sets the current gains
        case 'g'
            %query the gains
            kp = input('Provide a current kp gain: ');
            ki = input('Provide a current ki gain: ');
            %send both gains to pic
            fprintf(mySerial, '%f\n',kp);
            fprintf(mySerial, '%f\n',ki);
        %Reads/Gets the current gains
        case 'h'
            kp_rcvd = fscanf(mySerial, '%f');    %Read the kp
            ki_rcvd = fscanf(mySerial, '%f');    %Read the ki
            fprintf('Current Kp set to: %f\n', kp_rcvd);  %display kp
            fprintf('Current Ki set to: %f\n', ki_rcvd);  %display ki
        %Tests the current controls
        case 'k'
            data = read_plot_matrix(mySerial);
%             fprintf('Current Gains -- kp: %f | ki: %f',kp_rcvd, ki_rcvd);
        %Sets the position gains
        case 'i'
            %query the gains
            kpp = input('Provide a position kp gain: ');
            kip = input('Provide a position ki gain: ');
            kdp = input('Provide a position kd gain: ');
            %send both gains to pic
            fprintf(mySerial, '%f\n',kpp);
            fprintf(mySerial, '%f\n',kip);
            fprintf(mySerial, '%f\n',kdp);
        %Reads/Gets the position gains
        case 'j'
            kpp_rcvd = fscanf(mySerial, '%f');              %Read the kp
            kip_rcvd = fscanf(mySerial, '%f');              %Read the ki
            kdp_rcvd = fscanf(mySerial, '%f');              %Read the kd
            fprintf('Position Kp set to: %f\n', kpp_rcvd);  %display kp
            fprintf('Position Ki set to: %f\n', kip_rcvd);  %display ki
            fprintf('Position Ki set to: %f\n', kdp_rcvd);  %display kd
        %Go to angle (deg)
        case 'l'
            %query the desired degree
            deg = input('Provide the degree you want: ');
            %send the degree to the pic
            fprintf(mySerial, '%d\n', deg);
        %Load step trajectory
        case 'm'
            %query the user
            A = input('Enter step trajectory: ');   %e.g., "[0,0; 1,180; 3,-90; 4,0; 5,0]"
            refs = genRef(A, 'step');
            fprintf(mySerial, '%d', length(refs));  %send the length of the arr to the pic
            for i=1:length(refs)                    %send each of the samples
                fprintf(mySerial, '%d', refs(i));
            end
        %Load cubic trajectory
        case 'n'
            %query the user
            A = input('Enter cubic trajectory: ');   %e.g., "[0,0; 1,180; 3,-90; 4,0; 5,0]"
            refs = genRef(A, 'cubic');
            fprintf(mySerial, '%d', length(refs));  %send the length of the arr to the pic
            for i=1:length(refs)                    %send each of the samples
                fprintf(mySerial, '%d', refs(i));
            end
        case 'o'
            nsamples = fscanf(mySerial, '%d');
            data = zeros(nsamples,2);
            for i=1:nsamples
                data(i,:) = fscanf(mySerial, '%d %d');
                times(i) = (i-1)*5;
            end
            if nsamples > 1
                stairs(times, data(:,1:2));
            else
                fprintf('Only 1 sample received\n');
            end
            
            title(sprintf('Performance'));
            ylabel('Position');
            xlabel('Time (ms)');
        case 'q'
            has_quit = true;             % exit client
        otherwise
            fprintf('Invalid Selection %c\n', selection);
    end
end

end