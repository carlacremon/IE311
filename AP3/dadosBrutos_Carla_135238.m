

%%  Pulso FM =======================================================================
% ===============================================================================

radar  = Radar();

%parametros do pulso de transmissao
tau = 1e-6;                                                         %Duração do pulso
BW = 100e6;                                                        %Banda do pulso
M = fix(tau*radar.samplingFreq);                        %Número de amostras

pulse = chirpPulse(M,radar.samplingFreq,BW,tau);
pulse_zp_temp =  [pulse zeros(1, radar.N-M)];
shift = -floor(M/2);

% sinal de transmissao
pulsoFM = circshift(pulse_zp_temp, shift)/M; %shift de amostras

%% Simulador SAR====================================================================
% ===============================================================================

%cria 2 avos 
target1 = Target([230 radar.nearRange+2*radar.swathWidth/5 0],50, radar);
target2 = Target([270 radar.nearRange+3*radar.swathWidth/5 0],100, radar);


targets = [target1;target2];
dadosBrutos_antesComp = [];

%matriz de recepcao sem alvos
rxSignal = radar.rxNoTargetSignal;

% para cada instante de tempo
for t = 1:size(radar.position,1)
    
    % gera sinal de entrada do instante t, para cada alvo
    for alvo_idx = 1:length(targets)
        
        curr_target = targets(alvo_idx);
        
        % Ângulo entre velocidade e vetor (radar --> alvo)
        angleAlvo = acos(dot(radar.velocity/norm(radar.velocity),curr_target.rangeVector(t,:)/norm(curr_target.rangeVector(t,:))));
        
        %teste de visada
        isAlvoVisivel = angleAlvo > pi/2-radar.azmBeamwidth/2  && ...
            angleAlvo < pi/2+radar.azmBeamwidth/2;
        
        if isAlvoVisivel, rxSignal(end+1-t, curr_target.rangeBins(t)) = curr_target.signal(t); end
        
    end
    
    %    convolucao
     dadosBrutos_antesComp(end+1,:) = ifft(fft(rxSignal(t,:)) .* fft(pulsoFM));
    
end

% sinal antes da compressao

% figure;
% mesh(abs(dadosBrutos_antesComp));
% figure;
% imagesc(abs(dadosBrutos_antesComp));

%% Compressão em alcance=============================================================
% ===============================================================================
dadosBrutos_dpsComp = [];

%janelamento
m = -(M-1)/2:(M-1)/2;
hamming = 0.54 + 0.46*cos(2*pi*m/(M-1));
extendedHamming = [hamming zeros(1,N-M)];
shiftedHamming = circshift(extendedHamming,shift);

pulsoFMHamming = pulsoFM.*shiftedHamming;


% para cada instante de tempo
for t = 1:size(radar.position,1)
    
    %    compressao
     dadosBrutos_dpsComp(end+1,:) = ifft(fft(dadosBrutos_antesComp(t,:)) .* conj(fft(pulsoFMHamming)));
    
end

% sinal depois da compressao

% figure;
% mesh(abs(dadosBrutos_antesComp));
% figure;
% imagesc(abs(dadosBrutos_antesComp));



%% Funções aux =====================================================================
% ===============================================================================

function obj =  Radar()


c = physconst('LightSpeed');

% Parâmetros do radar -----------------------------------------------------

obj.theta=deg2rad(20);                                        % Ângulo entre o feixe da antena e a vertical/horizontal
obj.height=180;                                                   % H
obj.velocity = [100/3.6 0 0];                                % Velocidade
obj.azmBeamwidth = deg2rad(20);                       % Abertura da antena em azimute
obj.PRT = 1.515e-3;                                            % Tempo de repetição de pulso (pulse repetition time)
obj.azMax = 500;                                                 % Azimute máximo (fim trajetoria do aviao)
obj.lambda = 0.056;                                            % Comprimento de onda do radar
obj.N = 512;                                                        % Número de amostras em alcance
obj.samplingFreq = 120e6;                                  % Taxa de amostragem


obj.nearRange = obj.height*tan(obj.theta);          % Rgmin
obj.farRange = obj.height/tan(obj.theta);             % Rgmax
obj.swathWidth= obj.farRange - obj.nearRange;  % Lf
obj.deltaRange = c/obj.samplingFreq/2;             % Incremento em alcance

% position
x = 0:obj.velocity(1)*obj.PRT:obj.azMax;
y = zeros(size(x));
z = -obj.height*ones(size(x));

obj.position = [x' y' z'];                                        %posicao do radar em cada instante de tempo
obj.rxNoTargetSignal = zeros(length(x),obj.N);  % sinal de recepcao sem alvos


end


function obj = Target(position, RCS, radar)

obj.position = position;                                        %configura posicao do alvo
obj.RCS = RCS;
obj.rangeVector = obj.position - radar.position;   %Vetor que aponta do radar ao alvo para cada instante de tempo
obj.range = sqrt(sum(obj.rangeVector.^2,2));        % Distância: radar <--> alvo, para cada instante de tempo

phi = -4*pi*obj.range/radar.lambda;                     %fase

obj.signal = obj.RCS*exp(1i*phi);                           
obj.rangeBins = round(obj.range/radar.deltaRange) +1;



end

function  pulse = chirpPulse(M,samplingRate,BW,tau)

t = (-(M-1)/2: (M-1)/2)/samplingRate;

kappa = BW/tau;                         %Inclinação de f(t)
pulsePhase = pi*kappa*t.^2;             %Fase do pulso

pulse = exp(1i*pulsePhase);

end

function pulse = gaussianPulse(M)

m = -(M-1)/2:(M-1)/2;
alpha=16 ;                          % Parâmetro que controla a largura do pulso
sigma = M/(2*alpha);             % Desvio padrão
pulse = exp(-1/2*(m/sigma).^2);     % Pulso gaussiano


end