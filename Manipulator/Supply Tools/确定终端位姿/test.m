%%test
% addpath('F:\个人\学习历程\1.研究生涯\日常练习\空间机器人建模\practice\Supply Tools\Dynamics');
tic
[m, n] = size(Option_robot_state);
Option1 =zeros(2, n);%%%n
Option2 =zeros(2, n);%%%n
for i = 1:n%%%%n
    JJ = Kine_J(Option_robot_state(:, i), Robot_parameter);
    Option1(1, i) = det(JJ*JJ')^0.5;
    
    Jm = Kine_Jm(Option_robot_state(:, i), Robot_parameter);
    Option2(1, i) = det(Jm*Jm')^0.5;
    
    i
end
Option1(2,:) = 1:1:n;%%%n
Option2(2,:) = 1:1:n;%%%n

[~, OPT1] = max(Option1(1,:))
[~, OPT2] = max(Option2(1,:))

Option1 = fliplr(sortrows(Option1')');
Option2 = fliplr(sortrows(Option2')');

% Opti_num = 1000;
Opti_sol_in_JJ = zeros(6,n);
Opti_sol_in_Jm = zeros(6,n);

for i = 1:n
    Opti_sol_in_JJ(:, i) = Option_robot_state(:, Option1(2, i));
    Opti_sol_in_Jm(:, i) = Option_robot_state(:, Option2(2, i));
end

save 'Opti_sol_in_JJ.mat' Opti_sol_in_JJ;
save 'Opti_sol_in_Jm.mat' Opti_sol_in_Jm;

toc