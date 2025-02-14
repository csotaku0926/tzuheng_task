% **************** EXAMPLE OF A* ALGORITHM **********
clc
clear all
close all
% LOAD DATA
load('Example_data.mat');
END = posiz(1,:); %END POINT
START = posiz(end,:); %START POINT
feasible = 0;
i=1;
while feasible ~= 1 % RUN UNTIL A SOLUTION IS REACHED

    [cost(i), L]=A_STAR_new(posiz,size(posiz,1),factor);
    
    if cost(i) < Inf % CHECK ON THE COST VALUE
        feasible=1;
    else
        factor=2*factor; % increase nodes interconnections
        
        i=i+1; % iteration number 
    end
end
%% PLOT THE RESULT
figure
plot3(posiz(:,1),posiz(:,2),posiz(:,3)); % path wandered (blue)
hold on
back=plot3(posiz(L,1),posiz(L,2),posiz(L,3),'-r'); % shortest path (red)
set(back,'LineWidth',2,'Color','r')
hold on
plot3([END(1),START(1)],[END(2),START(2)],[END(3),START(3)],'+k','LineWidth',2)
grid on
text(END(1),END(2),END(3),'GOAL');
text(START(1),START(2),START(3),'START');

