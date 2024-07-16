syms t

scale = 2 / (3 - cos(2*t*(2*pi/60)));
x = 2 * scale * cos(t*(2*pi/60));
y = 2 * scale * sin(2*t*(2*pi/60)) / 2;

dotx = diff(x);
doty = diff(y);

v = sqrt(dotx^2+doty^2)
% (((2*pi*cos((pi*t)/15))/(15*(cos((pi*t)/15) - 3)) + (2*pi*sin((pi*t)/15)^2)/(15*(cos((t*pi)/15) - 3)^2))^2 + ((2*pi*sin((pi*t)/30))/(15*(cos((pi*t)/15) - 3)) - (4*pi*cos((pi*t)/30)*sin((pi*t)/15))/(15*(cos((t*pi)/15) - 3)^2))^2)^(1/2)



theta = atan2(doty,dotx)
% atan2(- (2*pi*cos((pi*t)/15))/(15*cos((pi*t)/15) - 45) - (2*pi*sin((pi*t)/15)^2)/(15*(cos((pi*t)/15) - 3)^2), (2*pi*sin((pi*t)/30))/(15*cos((pi*t)/15) - 45) - (4*pi*cos((pi*t)/30)*sin((pi*t)/15))/(15*(cos((pi*t)/15) - 3)^2))
