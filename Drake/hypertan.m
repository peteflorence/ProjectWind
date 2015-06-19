figure

scale = -3;
x = -5:0.01:5;
y = scale * (tanh(x)+1)/2;
plot(x,y), grid on