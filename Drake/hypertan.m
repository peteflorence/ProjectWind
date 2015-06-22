figure

scale = -3;
x = -5:0.01:5;
reversed = -1;
shift = 3;
y = scale * (tanh(reversed * (x-shift) * 10 ) +1) / 2;
plot(x,y), grid on




figure

scale = -3;
x = -5:0.01:5;
reversed = -1;
shift1 = -1;
y1 = scale * (tanh(reversed * (x-shift1) * 10 ) +1) / 2;

shift2 = 1;
y2 = - scale * (tanh(reversed * (x-shift2) * 10 ) +1) / 2;

y = y1 + y2;

plot(x,y), grid on

