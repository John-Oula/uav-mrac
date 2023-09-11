function dy=adaptive_algorithm(t,y)

    load data.mat A_LONG B_LONG; % Load A and B matrix
    
    dy=zeros(39,1); % dy: a vector of small increments of y during each time step
    
    % Adaption gains
    Gamma = 5* eye(2,2);
    gam = 1* eye(2,2); 
    disturbance = false;

    V_ref = 160; 
    theta_ref = 1;

%     if t < 30; V_ref = 200; end
%     if t > 30 && t < 60; V_ref = 100; end
%     if t > 60 && t < 90; V_ref = 200; end
%     if t > 90 && t < 120; V_ref = 100;end
%     if t > 120 && t < 150; V_ref = 200;end
%     if t > 150 && t < 180; V_ref = 100;end

    % Obtain u_ref from V_ref
    u_ref = V_ref*cos(theta_ref);
    r=[u_ref;theta_ref]; % Reference input
    
    % auxiliary signal \xi

    xi_1 = y(20)*y(8)+y(21)*y(10)+y(22)*y(12)+y(23)*y(14)+y(24)*y(16)+y(25)*y(18) - y(32);
    xi_2 = y(26)*y(8)+y(27)*y(10)+y(28)*y(12)+y(29)*y(14)+y(30)*y(16)+y(31)*y(18) - y(34);
    xi = [xi_1;xi_2]; 

    % auxiliary signal \epsilon

    eps_1=(y(1)-y(5))+y(36)* xi_1 + y(37)*xi_2;
    eps_2=(y(4)-y(6))+y(38)* xi_1 + y(39)*xi_2; 

    m = 1 + y(8)*y(8) + y(10)*y(10) + y(12)*y(12) + y(14)*y(14) + y(16)*y(16) + y(18)*y(18) + xi'*xi;

    % Controller Input

    mu_1 = y(20)*y(1)+y(21)*y(2)+y(22)*y(3)+y(23)*y(4)+ y(24)*r(1)+ y(25)*r(2);
    mu_2 = y(26)*y(1)+y(27)*y(2)+y(28)*y(3)+y(29)*y(4)+ y(30)*r(1)+ y(31)*r(2);
 
    

    % UAV dynamics
    if t > 70 && t < 170 && disturbance == true
%         mu = [saturate(mu_1,-20,20);saturate(mu_2,0,1)]; 
            mu=[mu_1+2*sin(t);mu_2];
        
    else
%         mu = [saturate(mu_1,-20,20);saturate(mu_2,0,1)];  
            mu=[mu_1;mu_2];
    end
    
    dy(1)= A_LONG(1,1)*y(1) + A_LONG(1,2)*y(2) + A_LONG(1,3)*y(3) + A_LONG(1,4)*y(4) + B_LONG(1,1)*mu(1) + B_LONG(1,2)*mu(2);
    dy(2)= A_LONG(2,1)*y(1) + A_LONG(2,2)*y(2) + A_LONG(2,3)*y(3) + A_LONG(2,4)*y(4) + B_LONG(2,1)*mu(1) + B_LONG(2,2)*mu(2);
    dy(3)= A_LONG(3,1)*y(1) + A_LONG(3,2)*y(2) + A_LONG(3,3)*y(3) + A_LONG(3,4)*y(4) + B_LONG(3,1)*mu(1) + B_LONG(3,2)*mu(2);
    dy(4)= A_LONG(4,1)*y(1) + A_LONG(4,2)*y(2) + A_LONG(4,3)*y(3) + A_LONG(4,4)*y(4) + B_LONG(4,1)*mu(1) + B_LONG(4,2)*mu(2);
    
    % Reference output ym

    dy(5) = -y(5) + r(1);
    dy(6) = y(7);
    dy(7) = -y(7) -y(6) +r(2); 

    % Auxiliary signals \zeta

    dy(8) = y(9);
    dy(9) = -y(9) -y(8) +y(1);

    dy(10) = y(11);
    dy(11) = -y(11) -y(10) +y(2);

    dy(12) = y(13);
    dy(13) = -y(13) -y(12) +y(3);

    dy(14) = y(15);
    dy(15) = -y(15) -y(14) +y(4);

    dy(16) = y(17);
    dy(17) = -y(17) -y(16) +r(1);

    dy(18) = y(19);
    dy(19) = -y(19) -y(18) +r(2);  

    % Lambda_1 dot

    dy(20) = -(Gamma(1,1)*eps_1*y(8))/m;
    dy(21) = -(Gamma(1,1)*eps_1*y(10))/m;
    dy(22) = -(Gamma(1,1)*eps_1*y(12))/m;
    dy(23) = -(Gamma(1,1)*eps_1*y(14))/m;
    dy(24) = -(Gamma(1,1)*eps_1*y(16))/m;
    dy(25) = -(Gamma(1,1)*eps_1*y(18))/m; 

    % Lambda_2  dot

    dy(26) = -(Gamma(2,2)*eps_2*y(8))/m;
    dy(27) = -(Gamma(2,2)*eps_2*y(10))/m;
    dy(28) = -(Gamma(2,2)*eps_2*y(12))/m;
    dy(29) = -(Gamma(2,2)*eps_2*y(14))/m;
    dy(30) = -(Gamma(2,2)*eps_2*y(16))/m;
    dy(31) = -(Gamma(2,2)*eps_2*y(18))/m;  

    % h(s) = \frac{1}{s^2+s+1}[mu](t)

    dy(32) = y(33);
    dy(33) = -y(33)-64*y(32) + mu_1; 

    dy(34) = y(35);
    dy(35) = -y(35)-64*y(34) + mu_2; 

    % rho_dot

    dy(36)=-(gam(1,1)*eps_1*xi_1)/m;
    dy(37)=-(gam(1,1)*eps_1*xi_2)/m;
    dy(38)=-(gam(2,2)*eps_2*xi_1)/m;
    dy(39)=-(gam(2,2)*eps_2*xi_2)/m; 




