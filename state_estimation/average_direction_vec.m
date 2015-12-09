function res_vec = average_direction_vec(v1,v2)
    %compute norm of v1 and v2 for debugging purposes
    %n1 = norm(v1);
    %n2 = norm(v2);
    %if ( abs(n1-1)>1e-5)
    %    disp 'v1 not normalized'
    %    %v1 = v1/n1;
    %end
    %if ( abs(n2-2)>1e-5)
    %    disp 'v2 not normalized'
    %    %v2 = v2/n2;
    %end
    %convert to spherical coordinates
    [a1,e1,r1] = cart2sph(v1(1),v1(2),v1(3));
    [a2,e2,r2] = cart2sph(v2(1),v2(2),v2(3));
    Ya = 0.5*(sin(a1)+sin(a2));
    Xa = 0.5*(cos(a1)+cos(a2));
    a = atan2(Ya,Xa);
    Ye = 0.5*(sin(e1)+sin(e2));
    Xe = 0.5*(cos(e1)+cos(e2));
    e = atan2(Ye,Xe);
    [vx,vy,vz] = sph2cart(a,e,1);
    res_vec = [vx,vy,vz];
    