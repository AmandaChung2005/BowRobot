function y = fftdeconv( num, den, highcut, lowcut, type )
% FFTDECONV  FFT-based deconvolution of two time-domain signals
%
%   y = FFTDECONV(NUM, DEN, HIGHCUT, LOWCUT, TYPE) returns a signal
%   determined using frequency-domain deconvolution of signals NUM by
%   DEN.  The optional parameters HIGHCUT and LOWCUT, given in the
%   range 0 (DC) - 1.0 (Nyquist), specify high and low frequency
%   cutoffs that are applied to the frequency-domain signals before
%   division and subsequent IFFT.  TYPE specifies a window type to
%   apply for the lowpass/ highpass filtering ('rectangular' or
%   'hanning').  When no type is specified, a rectangular window is
%   used.
%
%   By Gary P. Scavone, CCRMA, Stanford University, July 2001.
%   Updated May 2008, McGill University.

if ( nargin < 2 | nargin > 5 ),
  error('Number of arguments is incorrect.');
  return
end

N = size(num);
D = size(den);

if ( N(1) ~= 1 | D(1) ~= 1 ),
  error('Input signals must be row vectors.');
  return
end

L = N(2) + D(2);

num = [num, zeros(1,L-N(2))];
den = [den, zeros(1,L-D(2))];

LC = -1;
HC = -1;

if ( nargin < 5 )
  type = 'none';
end

if ( nargin > 3 )
  % Low frequency shelf
  if lowcut < 0 | lowcut >= 1 | lowcut > highcut
    error('LOWCUT argument error.');
    return
  end
  if lowcut > 0
    LC = round(lowcut * 0.5 * L);
  end
end

if ( nargin > 2 )
  % High frequency shelf
  if highcut <= 0 | highcut > 1
    error('HIGHCUT argument error.');
    return
  end
  if highcut < 1.0,
    HC = round(highcut * 0.5 * L);
  end
end

NUM = fft(num);
DEN = fft(den);

% Apply lowcut, if necessary
if ( LC > 0 )
  if strcmp( type, 'hanning' )
    window = hann( 2 * LC );
    NUM(1:LC) = NUM(1:LC) .* window(1:LC).';
    DEN(1) = 1.0;
  else
    NUM(1:LC) = 0.0;
    DEN(1:LC) = 1.0;
  end
  %DEN(1:LC) = 1.0;
end

% Apply highcut, if necessary
if ( HC > 0 )
  if strcmp(type, 'hanning')
    W = floor(L/2) - HC +1;
    window = hann(2*W);
    NUM(HC+1:floor(L/2)+1) = NUM(HC+1:floor(L/2)+1).*window(W+1:2*W).';
    DEN(floor(L/2)+1) = 1.0;
  else
    NUM(HC+1:floor(L/2)+1) = 0.0;
    DEN(HC+1:floor(L/2)+1) = 1.0;
  end
  %DEN(HC+1:floor(L/2)+1) = 1.0;
end

Y = NUM ./ DEN;

LD2 = floor(0.5 * L);
Y = [Y(1:LD2+1), conj(fliplr(Y(2:LD2)))];

y = real( ifft(Y) );


