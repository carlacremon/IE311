function pulse = gaussianPulse(M)

m = -(M-1)/2:(M-1)/2;
alpha=16 ;                          % Parâmetro que controla a largura do pulso
sigma = M/(2*alpha);             % Desvio padrão
pulse = exp(-1/2*(m/sigma).^2);     % Pulso gaussiano


end
