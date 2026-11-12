function a = wrapPi(a)
%WRAPPI Wrap angle to (-pi, pi].
a = mod(a + pi, 2*pi) - pi;
end
