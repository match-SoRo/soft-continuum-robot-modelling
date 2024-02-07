function X = hat(x)
if numel(x)==3
    X=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
else
    X=[0 -x(3+3) x(2+3) x(1); x(3+3) 0 -x(1+3) x(2); -x(2+3) x(1+3) 0 x(3); 0 0 0 0];
end
end
