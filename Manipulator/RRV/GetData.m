% 多次运行Main函数(RRV)  
% Copyrigt by Peng Cai 
% 01/29/2021
clear
clc

Num = 100;
Result = zeros(Num, 7);
for kk = 1:Num
    
    Result(kk, :) = Main;

	disp(['已经进行第 ' num2str(kk) ' 次迭代！！！！！！！']);

    
end



