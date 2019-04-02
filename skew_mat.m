function skewed_mat = skew_mat(w)
%This will give the skew symmetirc matrix of the vector w which can be used
%to calculate the cross product with another vector v

skewed_mat = [ 0     -w(3)  w(2);
               w(3)   0    -w(1);
              -w(2)   w(1)  0];

end

