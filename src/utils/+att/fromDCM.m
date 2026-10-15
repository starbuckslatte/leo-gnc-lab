function q = fromDCM(A)
%ATT.FROMDCM Quaternion from attitude matrix using Shepperd's method.
%   A maps reference to body (v_body = A*v_ref). Selects the numerically
%   largest component to avoid division by a small number (robust near
%   180 deg rotations). Returns scalar-first q with q(1) >= 0.
%   Equivalent to EGSE M2Q(A').
tr = trace(A);
[~, k] = max([tr, A(1,1), A(2,2), A(3,3)]);
switch k
    case 1
        qs = 0.5*sqrt(1 + tr);
        q  = [qs;
              (A(2,3) - A(3,2)) / (4*qs);
              (A(3,1) - A(1,3)) / (4*qs);
              (A(1,2) - A(2,1)) / (4*qs)];
    case 2
        q1 = 0.5*sqrt(1 + 2*A(1,1) - tr);
        q  = [(A(2,3) - A(3,2)) / (4*q1);
              q1;
              (A(1,2) + A(2,1)) / (4*q1);
              (A(1,3) + A(3,1)) / (4*q1)];
    case 3
        q2 = 0.5*sqrt(1 + 2*A(2,2) - tr);
        q  = [(A(3,1) - A(1,3)) / (4*q2);
              (A(1,2) + A(2,1)) / (4*q2);
              q2;
              (A(2,3) + A(3,2)) / (4*q2)];
    case 4
        q3 = 0.5*sqrt(1 + 2*A(3,3) - tr);
        q  = [(A(1,2) - A(2,1)) / (4*q3);
              (A(1,3) + A(3,1)) / (4*q3);
              (A(2,3) + A(3,2)) / (4*q3);
              q3];
end
if q(1) < 0
    q = -q;
end
q = att.normalize(q);
end
