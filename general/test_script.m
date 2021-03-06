%% PARAMS

dPRF = 1700;    %Hz
dFreqInicial = 1e9; %GHz
dLarguraBanda = 5e6;%MHz;
dFreqAmostragem = 100e6; %MHz
dDuracaoPulso = 100e-6; %s
dTempoInicial = 0;

dFaseInicial = 0; 
dAmplitude = 1;


t = [dTempoInicial:1/dFreqAmostragem:dDuracaoPulso];
f = dFreqInicial:(dFreqInicial+dLarguraBanda);
%% chirp matlab
y = dAmplitude*chirp(t,dFreqInicial,dDuracaoPulso,dFreqInicial+dLarguraBanda)

%% meu chirp
% z = meu_chirp(t,f0,B)
