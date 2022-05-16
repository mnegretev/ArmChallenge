function [Q,T] = get_polynomial_trajectory(q0, q1, t, dt)
    A = [   t^5,    t^4,   t^3, t^2,  t, 1;
          5*t^4,  4*t^3, 3*t^2,  2*t, 1, 0;
         20*t^3, 12*t^2,   6*t,    2, 0, 0;
              0,      0,     0,    0, 0, 1;
              0,      0,     0,    0, 1, 0;
              0,      0,     0,    2, 0, 0];
    B = [q1;0;0;q0;0;0];
    X = A\B;
    a5 = X(1);
    a4 = X(2);
    a3 = X(3);
    a2 = X(4);
    a1 = X(5);
    a0 = X(6);
    T = [0:dt:t];
    Q = zeros(1, length(T));
    for i=1:length(T)
        Q(i) = a5*T(i)^5 + a4*T(i)^4 + a3*T(i)^3 + a2*T(i)^2 + a1*T(i) + a0;
    end
    
    