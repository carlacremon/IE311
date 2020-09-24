function adSinal = meu_chirp(dAmplitude,adTempo,dFreqInicial,dFaseInicial,k)

adSinal=dAmplitude*cos(2*pi*(k/2*adTempo+dFreqInicial).*adTempo+dFaseInicial);
end