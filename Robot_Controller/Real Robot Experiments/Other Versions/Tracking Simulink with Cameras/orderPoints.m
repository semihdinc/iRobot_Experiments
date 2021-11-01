function new_p = orderPoints(p)
%#codegen
    [~, indx] = sort(p(1,:));
    [~, indy] = sort(480-p(2,:));

    ind4 = indy(4);
    ind3 = indy(3);

    ind1 = indx(1);
    ind2 = indx(4);

    new_p = p(:,[ind1 ind2 ind3 ind4]);
end