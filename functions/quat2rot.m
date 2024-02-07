function R = quat2rot(h)
% R = zeros(3);
h = h/norm(h);
h1=h(1); h2=h(2); h3=h(3); h4=h(4);
R = eye(3) + 2/(h'*h) *...
    [-h3^2-h4^2, h2*h3-h4*h1, h2*h4+h3*h1;
    h2*h3+h4*h1, -h2^2-h4^2, h3*h4-h2*h1;
    h2*h4-h3*h1, h3*h4+h2*h1, -h2^2-h3^2 ];
end

