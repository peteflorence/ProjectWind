figure

scale = -3;
x = -5:0.01:5;
reversed = -1;
shift = 3;
y = scale * (tanh(reversed * (x-shift) * 10 ) +1) / 2;
plot(x,y), grid on

