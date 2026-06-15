function [ir,t] = modalIr(freq,damp,amp,fs,dur)
% Function to take in frequencies, amplitudes, damping, sample rate, and
% duration, and give back a modal impulse response


    N = round(fs*dur);
    t = ((0:N-1)/fs);

    omega = 2*pi*freq;
    s = -omega .* damp + 1i *omega;
    basis = exp(s*t);

    ir = real(basis'*amp);

end