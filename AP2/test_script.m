% Limites de alcance no solo ----------------------------------------------

radar = Radar();

%position                    %RCS
target1 = Target([230 radar.nearRange+2*radar.swathWidth/5 0],50, radar);
target2 = Target([270 radar.nearRange+3*radar.swathWidth/5 0],100, radar);

targets = [target1;target2];


circConv = [];
rxSignal = radar.rxNoTargetSignal;

%% para cada instante de tempo
for t = 1:size(radar.position,1)
    
    %% gera sinais de entrada, para cada alvo
    for alvo_idx = 1:length(targets)
        
        curr_target = targets(alvo_idx);
        % Ângulo entre velocidade e vetor (radar --> alvo)
        angleAlvo = acos(dot(radar.velocity/norm(radar.velocity),curr_target.rangeVector(t,:)/norm(curr_target.rangeVector(t,:))));
        
        isAlvoVisivel = angleAlvo > pi/2-radar.azmBeamwidth/2  && ...
            angleAlvo < pi/2+radar.azmBeamwidth/2;
        
        if isAlvoVisivel, rxSignal(end+1-t, curr_target.rangeBins(t)) = curr_target.signal(t); end
        
    end
    %% convolucao
    circConv(end+1,:) = ifft(fft(rxSignal(t,:)) .* fft(radar.pulse_zp));
    
end


figure(1)
plot(radar.pulse_zp)
title('Sinal de entrada')
xlim([0 radar.N])

figure(3)
subplot(2,2,1)
mesh(abs(circConv))
title('Magnitude')

figure(2)
subplot(2,2,1)
imagesc(abs(circConv))
title('Magnitude')

subplot(2,2,3)
imagesc(rad2deg(angle(circConv)))
title('Fase (graus)')

subplot(2,2,2)
imagesc(real(circConv))
title('Parte real')

subplot(2,2,4)
imagesc(imag(circConv))
title('Parte imaginária')

