%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                          %
%           Generated by MATLAB 9.4 and Fixed-Point Designer 6.1           %
%                                                                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function adSinal = meu_chirp_wrapper_fixpt(dAmplitude,adTempo,dFreqInicial,dFaseInicial,k)
    fm = get_fimath();
    dAmplitude_in = fi( dAmplitude, 0, 1, 0, fm );
    adTempo_in = fi( adTempo, 0, 1, 4, fm );
    dFreqInicial_in = fi( dFreqInicial, 0, 30, 0, fm );
    dFaseInicial_in = fi( dFaseInicial, 0, 1, 0, fm );
    k_in = fi( k, 0, 36, 0, fm );
    [adSinal_out] = meu_chirp_fixpt( dAmplitude_in, adTempo_in, dFreqInicial_in, dFaseInicial_in, k_in );
    adSinal = double( adSinal_out );
end

function fm = get_fimath()
	fm = fimath('RoundingMethod', 'Floor',...
	     'OverflowAction', 'Wrap',...
	     'ProductMode','FullPrecision',...
	     'SumMode','FullPrecision');
end
