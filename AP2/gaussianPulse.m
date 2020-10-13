function pulse = gaussianPulse(M)

m = -(M-1)/2:(M-1)/2;
alpha=16 ;                          % Par�metro que controla a largura do pulso
sigma = M/(2*alpha);             % Desvio padr�o
pulse = exp(-1/2*(m/sigma).^2);     % Pulso gaussiano


end
