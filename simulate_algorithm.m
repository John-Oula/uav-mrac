function simulate_algorithm()

    % auxiliary signals
    zeta = zeros(12,1);
    lambda = 0.05 - [ 2 -1 -0.5 -0.6 -2 -0.002 -0.2 0.6 0.15 0.2 0.2 0];   
    rho = zeros(4,1);

   % initial condition
  
   y0=[
       60 4 0 0 ... % states y(1:4)'
       60 ...% y_m_1,
       0 0 ... % y_m_2
       zeta' ... y(8:19)
       lambda ...y(20:31)
       0 0 0 0 ... % \frac{1}{s^2+s+1}[\LambdaˆT\omega](t) y(32:35)
       rho'... % y(36:39)
       ]; 

   [t,y]=ode45("adaptive_algorithm",[0 200],y0); % ode function

   % y: an array of solutions to a set of differential equations
   % each row of y corresponds to a time instant returned in vector t
   % in this case, y=[state x (u,w,q,theta), reference output y_m,
   % auxiliary signals \Lambda
   % and \frac{1}{s^2+s+1}[\LambdaˆT\omega](t), parameter estimates \Lambda and \rho]ˆT

   save simulation_data t y; % save data
   save result.m y

