classdef OptTraj < handle
    %   求解最优激励轨迹
    %   此处显示详细说明
    
    properties
%         x = sym('x',[1,2])
        x = sym('x')
        y = sym('y')
        lb = [0,0.2]
        ub = [0.5,0.8]
        A = [1 2; 2 1]
        b = [1; 1]
        Aeq = []
        beq = []
        x0 = [1/4 1/4]
    end
   
    
    methods
        function fun = tarfun(obj)
            fun = 100*(obj.y-obj.x^2)^2 + (1-obj.x)^2;
        end
        
        function [c,ceq] = circlecon(obj)
            c = (obj.x-1/3)^2 + (obj.y-1/3)^2 -(1/3)^2;
            ceq = [];
        end

        function var = optimal(obj)
            myfun = tarfun(obj);
            fun = matlabFunction(myfun,'Vars',{[obj.x obj.y]});
            f1 = matlabFunction(myfun);
            f2 = matlabFunction(myfun,'Vars',[obj.x obj.y]);
            mycon = circlecon(obj);
            nonlcon = matlabFunction(mycon);
%             var = fmincon(fun,obj.x0,obj.A,obj.b,obj.Aeq,obj.beq,obj.lb,obj.ub,nonlcon);
            var = fmincon(fun,obj.x0,obj.A,obj.b);
        end
    end
        
            
end

