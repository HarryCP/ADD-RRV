% �������Main����(RRV)  
% Copyrigt by Peng Cai 
% 01/29/2021
clear
clc

Num = 30;
Result = zeros(Num, 7);
parfor kk = 1:Num
    
    Result(kk, :) = Main;

	disp(['�Ѿ����е� ' num2str(kk) ' �ε�����������������']);

    
end



