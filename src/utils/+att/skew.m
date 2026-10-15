function S = skew(v)
%ATT.SKEW Cross-product (skew-symmetric) matrix: skew(a)*b == cross(a, b).
S = [  0    -v(3)   v(2);
      v(3)    0    -v(1);
     -v(2)   v(1)    0  ];
end
