classdef Radar
    
    properties
        theta=deg2rad(20);  % Ângulo entre o feixe da antena e a vertical/horizontal
        height=180;                         % H
        
        
        nearRange;      % Rgmin
        farRange;       % Rgmax
        swathWidth;  % Lf
        
        % Parâmetros do radar -----------------------------------------------------
        velocity = [100/3.6 0 0];               % Velocidade
        azmBeamwidth = deg2rad(20);     % Abertura da antena em azimute
        PRT = 1.515e-3;                         % Tempo de repetição de pulso (pulse repetition time)
        azMax = 500;                                % Azimute máximo (fim trajetoria do aviao)
        
        lambda = 0.056;                     % Comprimento de onda do radar
        N = 512;                                % Número de amostras em alcance
        samplingFreq = 120e6;           % Taxa de amostragem
        
        position; %posicao do radar ao longo do tempo
        deltaRange;  % Incremento em alcance
        rxNoTargetSignal; % sinal de recepcao sem alvos
        
        %- pulse
        M = 64 % Largura da janela
        pulse_zp;
    end
    
    methods
        function  obj = Radar()
            c = physconst('LightSpeed');
            
            obj.nearRange = obj.height*tan(obj.theta);      % Rgmin
            obj.farRange = obj.height/tan(obj.theta);       % Rgmax
            obj.swathWidth = obj.farRange - obj.nearRange;  % Lf
            obj.deltaRange = c/obj.samplingFreq/2;  % Incremento em alcance
            
            x = 0:obj.velocity(1)*obj.PRT:obj.azMax;
            y = zeros(size(x));
            z = -obj.height*ones(size(x));
            
            obj.position = [x' y' z'];
        
            
            % sinal de recepcao sem alvos
            obj.rxNoTargetSignal = zeros(length(x),obj.N);
            
            
            %pulso de transmissao
            pulse = gaussianPulse(obj.M);
            pulse_zp_temp =  [pulse zeros(1, obj.N-obj.M)];
            
            shift = -floor(obj.M/2); 
            obj.pulse_zp = circshift(pulse_zp_temp, shift); %shift de amostras
            
        end
        

        end
    end