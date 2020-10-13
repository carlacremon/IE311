function rxSignal = generateRxSignal(radar, targets)

rxSignal = radar.rxNoTargetSignal;

%para cada instante t da trajetoria da aeronave
for t = 1:size(radar.position,1)
    
    
    for alvo_idx = 1:length(targets)
        
        curr_target = targets(alvo_idx);
        
        % Ângulo entre velocidade e vetor (radar --> alvo)
        angleAlvo = acos(dot(radar.velocity/norm(radar.velocity),curr_target.rangeVector(t,:)/norm(curr_target.rangeVector(t,:))));
        
        isAlvoVisivel = angleAlvo > pi/2-radar.azmBeamwidth/2  && ...
            angleAlvo < pi/2+radar.azmBeamwidth/2;
        
        if isAlvoVisivel, rxSignal(end+1-t, curr_target.rangeBins(t)) = curr_target.signal(t); end
        
    end
    
end



end