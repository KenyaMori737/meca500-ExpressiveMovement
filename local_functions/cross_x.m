function [out] = cross_x(in)
    
    if sum(size(in))==4
        vec = in;

        out = zeros(3,3);
        out(1,2) = -vec(3,1); out(1,3) =  vec(2,1); out(2,3) = -vec(1,1);
        out(2,1) =  vec(3,1); out(3,1) = -vec(2,1); out(3,2) =  vec(1,1);

    else
        mat = in;

        out = zeros(3,1);
        out(1,1) = -mat(2,3);
        out(2,1) =  mat(1,3);
        out(3,1) = -mat(1,2);
        
    end
    
end