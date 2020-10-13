classdef Target
    
    properties
        position
        RCS
        rangeVector;
        range;
        signal;
        rangeBins;
    end
    
    methods
        
        function obj = Target(position, RCS, radar)
            
            obj.position = position;
            obj.RCS = RCS;  
            obj.rangeVector = obj.position - radar.position; 
            obj.range = sqrt(sum(obj.rangeVector.^2,2));  % Distância: radar <--> alvo
            
            phi = -4*pi*obj.range/radar.lambda;
        
            obj.signal = obj.RCS*exp(1i*phi);
            obj.rangeBins = round(obj.range/radar.deltaRange) +1;
                     
                                        
        end
        
    end
    
end