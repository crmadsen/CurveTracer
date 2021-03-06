%%  Transistor Identification and Characterization System
%
%  MATLAB Display Code
%
%  7 April 2016
%
%  Created by Luis Garc�a
%  Copyright LGarcia 2016  ***Use with Permission***
%
%---- Set Up MATLAB
clc, clear, close all;
format long;

%% Reading the File

% Open File for reading

% Due to the way the University sets up network accounts for all staff and
% students it is impossible to anticipate the location of the comma
% separated file being imported. As such the user will be prompted to
% enter the path of the file as the first step in the execution of this 
% script. The path must correspond to the location where MATLAB is 
% configured, in order to search for data files. It should be in a format 
% similar to the following:
%
%   drive:\Users\username\Documents\MATLAB\curve.csv
%   e.g. C:\Users\jjones\Documents\MATLAB\curve.csv
% 
% Ensure to include the .csv in te pat name.
% The easiest thing may be to open a file explorer window, find the file, &
% copy the path from the file explorer window.

%****************REMEMBER TO UNCOMMENT WHEN FINAL************************
file_path = input('Enter Data File Path: ','s')
fid = fopen(file_path,'r');
%************************************************************************

% Read Transistor Identification from File
formatSpec = '%s';
N = 2;
Id_text = textscan(fid,formatSpec,N,'Delimiter',',');
formatSpec = '%s';
N = 3;
Id_text2 = textscan(fid,formatSpec,N,'Delimiter',',');

% Read Curve Tracer Data
curve_data = textscan(fid,'%f %9.6f %9.6f', 'Headerlines', 3,...
    'Delimiter', ',', 'MultipleDelimsAsOne', 3, 'ReturnOnError', 0);

fclose(fid);

% Define Identification variables

% These next lines extract the header information in the Comma Separated
% File generated by the system 
% Id_text is the string containing all header information 

t_type_raw = Id_text{1}(1);
t_type = {t_type_raw{1}(6:end)};
t_subtype_raw = Id_text{1}(2);
t_subtype = {t_subtype_raw{1}(9:end)};
t_terminal_1 = Id_text2{1}(1);
t_terminal_2 = Id_text2{1}(2);
t_terminal_3 = Id_text2{1}(3);


% These next lines extract the values of the gate/base steps used in the
% plots, which is needed for accurate plotting.

% Using the first column, "curve_data{1}", of the Comma Separated File
% the first step value is taken to be the first value found in the column
% "curve_data{1}(1)". Then a find statement finds the index values 
% of all cells with a value larger than the first step value, and stores 
% them in the matrix: next_vgs. The next step value is then equal to the
% value associated with the first index listed in next_vgs. The process
% is repeated until all 6 steps are found.  A similar process is used for
% the "p" devices where the csv data is stored in decreasing order instead
% of increasing order.  The if statement checks to see if the data is
% increasing or decreasing and the intervals are created accordingly.

first_vgs_value = curve_data{1}(1);
which_way = find(curve_data{1}~=first_vgs_value);
if (curve_data{1}(which_way(1))>first_vgs_value)
    next_vgs = find(curve_data{1}>first_vgs_value);
    second_vgs_value = curve_data{1}(next_vgs(1));
    next_vgs = find(curve_data{1}>second_vgs_value);
    third_vgs_value = curve_data{1}(next_vgs(1));
    next_vgs = find(curve_data{1}>third_vgs_value);
    fourth_vgs_value = curve_data{1}(next_vgs(1));
    next_vgs = find(curve_data{1}>fourth_vgs_value);
    fifth_vgs_value = curve_data{1}(next_vgs(1));
    next_vgs = find(curve_data{1}>fifth_vgs_value);
    sixth_vgs_value = curve_data{1}(next_vgs(1));
else
    next_vgs = find(curve_data{1}<first_vgs_value);
    second_vgs_value = curve_data{1}(next_vgs(1));
    next_vgs = find(curve_data{1}<second_vgs_value);
    third_vgs_value = curve_data{1}(next_vgs(1));
    next_vgs = find(curve_data{1}<third_vgs_value);
    fourth_vgs_value = curve_data{1}(next_vgs(1));
    next_vgs = find(curve_data{1}<fourth_vgs_value);
    fifth_vgs_value = curve_data{1}(next_vgs(1));
    next_vgs = find(curve_data{1}<fifth_vgs_value);
    sixth_vgs_value = curve_data{1}(next_vgs(1));
end

% Sub-dividing the Curve File into respective Step Voltages

% Once the steps have been found the indices for all comma separated file
% readings corresponding to each step value are determined. This is done
% to enable easy plotting of each of the steps. The find command scans the
% first column of the comma separated file, "curve_data{1}", for those
% entries that match the specific step value. Those values are then
% assigned to their particular matrix.

first_vgs = find(curve_data{1}==first_vgs_value);
second_vgs = find(curve_data{1}==second_vgs_value);
third_vgs = find(curve_data{1}==third_vgs_value);
fourth_vgs = find(curve_data{1}==fourth_vgs_value);
fifth_vgs = find(curve_data{1}==fifth_vgs_value);
sixth_vgs = find(curve_data{1}==sixth_vgs_value);

%% Set up Plot

figure
% The six commands below take the data from the device and runs it through
% a moving average calculatin to smooth out any ripples that the system
% algorithm failed to catch

curve_data{3}(first_vgs) = smooth(curve_data{3}(first_vgs),9);
curve_data{3}(second_vgs) = smooth(curve_data{3}(second_vgs),9);
curve_data{3}(third_vgs) = smooth(curve_data{3}(third_vgs),9);
curve_data{3}(fourth_vgs) = smooth(curve_data{3}(fourth_vgs),9);
curve_data{3}(fifth_vgs) = smooth(curve_data{3}(fifth_vgs),9);
curve_data{3}(sixth_vgs) = smooth(curve_data{3}(sixth_vgs),9);

% Plot the first curve

plot(curve_data{2}(sixth_vgs),curve_data{3}(sixth_vgs),'c',...
   'LineWidth',2)
set(gca,'FontSize',16) %sets the axis and legend to a specific size
 hold on;

% The next set of commands plots each of the remaining curves
plot(curve_data{2}(fifth_vgs),curve_data{3}(fifth_vgs),'m',...
   'LineWidth',2)
plot(curve_data{2}(fourth_vgs),curve_data{3}(fourth_vgs),'g',...
   'LineWidth',2)
plot(curve_data{2}(third_vgs),curve_data{3}(third_vgs),'b',...
   'LineWidth',2)
plot(curve_data{2}(second_vgs),curve_data{3}(second_vgs),'k',...
   'LineWidth',2)
plot(curve_data{2}(first_vgs),curve_data{3}(first_vgs),'r',...
   'LineWidth',2)

        
% generates the minor scale grid. The minor scale grid isn't a default
% setting so an additional set of commands is necessary.

set(gca, 'GridLineStyle', '-');
grid(gca,'minor')
title('grid minor')

% Formating the Plot

% The plot is going to be different based on the type of transistor, so an
% if statement is set up to ensure the correct plot is generated. The
% condition for the if statement is set up by using the string compare
% command, strcmp, which returns a 1 if the t_type matches 'MOSFET', and a 
% 0 if it doesn't.  

tf = strcmp(' MOSFET',t_type);
if (tf == 1)
    
    % Formatting for MOSFET

    tsf_m = strcmp(' PMOS',t_subtype);
    if (tsf_m == 0)
        xlabel('V_{DS} (Volts)','FontSize',16,'FontWeight','bold')
    else
        xlabel('V_{SD} (Volts)','FontSize',16,'FontWeight','bold')
    end
    
    ylabel('I_{D} (Amp)','FontSize',16,'FontWeight','bold')
    
    % Legend Creation
    
    % The legend entries need to be generated individually because they
    % are dynamic based on the step values found earlier in the code. The
    % sprintf command generates a string using the appropriate step value,
    % which is then used in the actual legend statement.
    
        Legend_entry_1 = sprintf('V_{G}= %1.1fV (Top Curve)',...
            sixth_vgs_value);
        Legend_entry_2 = sprintf('V_{G}= %1.1fV',fifth_vgs_value);
        Legend_entry_3 = sprintf('V_{G}= %1.1fV',fourth_vgs_value);
        Legend_entry_4 = sprintf('V_{G}= %1.1fV',third_vgs_value);
        Legend_entry_5 = sprintf('V_{G}= %1.1fV',second_vgs_value);
        Legend_entry_6 = sprintf('V_{G}= %1.1fV (Bottom Curve)',...
            first_vgs_value);
        legend(Legend_entry_1, Legend_entry_2, Legend_entry_3,...
            Legend_entry_4, Legend_entry_5, Legend_entry_6,...
            'Location','EastOutside');
    
    % Title Sequence Creation
    
    % The title is also dynamic based on the type of transistor. Using a
    % methodology similar to the legend, two sub-title strings are generated
    % using sprintf commands. The first string, sub_title1, includes
    % the transistor's type and subtype stored as the variables
    % t_subtype and t_type shown earlier in the code. 
    % The second string, sub_title2, contains the Terminal Identification
    % Information from the comma separated file, which were stored in 
    % t_terminal_1{1},t_terminal_2{1}, and t_terminal_3{1}.

    sub_title1 = sprintf('%s %s Transistor',t_subtype{1},t_type{1});
    sub_title2 = sprintf(' %s /  %s /  %s\n',...
        t_terminal_1{1},t_terminal_2{1},t_terminal_3{1});
    
    title(['\fontsize{20pt}\bf{Drain Current vs. Drain-Source Voltage}'...
        char(10) '\fontsize{14pt}\rm' sub_title1 char(10)...
        '\fontsize{12pt}\rm' sub_title2])

else
 
    % Formatting for BJT
    
    % The MOSFET formatting logic was adapted for the BJT below
    
    tsf_b = strcmp(' PNP',t_subtype); 
    if (tsf_b == 0)
        xlabel('V_{CE} (Volts)','FontSize',16,'FontWeight','bold')
    else
        xlabel('V_{EC} (Volts)','FontSize',16,'FontWeight','bold')
    end
    ylabel('I_{C} (Amp)','FontSize',16,'FontWeight','bold')
    
     % Legend Creation
    
    %tsf = strcmp(' PNP',t_subtype);
%     if (tsf == 0)
        Legend_entry_1 = sprintf('V_{B}= %1.1fV (Top Curve)',...
            sixth_vgs_value);
        Legend_entry_2 = sprintf('V_{B}= %1.1fV',fifth_vgs_value);
        Legend_entry_3 = sprintf('V_{B}= %1.1fV',fourth_vgs_value);
        Legend_entry_4 = sprintf('V_{B}= %1.1fV',third_vgs_value);
        Legend_entry_5 = sprintf('V_{B}= %1.1fV',second_vgs_value);
        Legend_entry_6 = sprintf('V_{B}= %1.1fV (Bottom Curve)',...
            first_vgs_value);
        legend(Legend_entry_1, Legend_entry_2, Legend_entry_3,...
            Legend_entry_4, Legend_entry_5, Legend_entry_6,...
            'Location','EastOutside');
    
    % Title Sequence Creation
    
    sub_title1 = sprintf('%s %s Transistor',t_subtype{1},t_type{1});
    sub_title2 = sprintf(' %s / %s / %s\n',...
        t_terminal_1{1},t_terminal_2{1},t_terminal_3{1});
    title(['\fontsize{20pt}\bf{Collector Current vs. Collector-Emitter Voltage}'...
        char(10) '\fontsize{14pt}\rm' sub_title1 char(10)...
        '\fontsize{12pt}\rm' sub_title2])
end

% The next command resizes the plot for a more comfortable viewing

set(gcf,'position',[50 100 1200 800])  %increases the size of the figure

fclose('all');