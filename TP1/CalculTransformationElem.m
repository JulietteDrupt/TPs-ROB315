function [g] = CalculTransformationElem(alphai,di,thetai,ri)
    g = [cos(thetai) -sin(thetai) 0 di;
        cos(alphai)*sin(thetai) cos(alphai)*cos(thetai) -sin(alphai) -ri*sin(alphai);
        sin(alphai)*sin(thetai) sin(alphai)*cos(thetai) cos(alphai) ri*cos(alphai);
        0 0 0 1];
end