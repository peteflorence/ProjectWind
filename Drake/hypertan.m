function hypertan()

%simple();
%doubleShifted();

radius();


end


function simple()
figure

scale = -3;
x = -5:0.01:5;

reversed = -1;
shift = 3;
y = scale * (tanh(reversed * (x-shift) * 10 ) +1) / 2;
plot(x,y), grid on

end



function doubleShifted()
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
end

function radius()

figure

scale = -3;
x = -10:0.01:10;

reversed = -1;

a = sqrt((x-0).^2);

Radius = 7;
slope = 10;

y = scale * (tanh(reversed * ( a - Radius) * slope ) +1) / 2;
plot(x,y), grid on



end

