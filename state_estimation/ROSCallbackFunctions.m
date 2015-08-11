classdef ROSCallbackFunctions <handle
    properties
        vals % vector of values which can be updated 
        offsets = [3.86-0.15; 3.97; 3.92+0.5; 3.95; 3.92+0.4];
    end
    
    methods
        function obj =  ROSCallbackFunctions(vals)
            obj.vals = vals; %initialize values
        end
        
        function updateVal(obj,msg)
            val = msg.Data;
            obj.vals(:,1) = val-obj.offsets; %update vals at the ith index
            obj.vals(:,2) = val>0.1 & val<10;
            %disp(obj.vals)
        end
        
        function timerUpdate(obj,funcHandle)
            %this is a function wrapper for an update function
            vec = obj.vals;
            funcHandle(vec);
            obj.vals(:,2) = 0;
        end
    end    
end

