function simu_plot(v)
    load simulation_data.mat y t; % load simulation data
    
    subplot(2,1,1);
    plot(t,y(:,1),t,y(:,5),'LineWidth',2);
    xlabel ("$t(s)$",'interpreter','latex','FontSize',14,'FontUnits','points');
    ylabel ("$u$ [m/s]",'interpreter','latex','FontSize',14,'FontUnits','points');
    grid on
    legend('$u$','$u_{ref}$','interpreter','latex','FontSize',14);
    
    subplot(2,1,2);
    plot(t,y(:,4),t,y(:,6),'LineWidth',2);
    xlabel ("t(s)",'interpreter','latex','FontSize',14,'FontUnits','points');
    ylabel ("$\theta$ [deg]",'interpreter','latex','FontSize',14,'FontUnits','points');
    legend('$\theta$','$\theta_{ref}$','interpreter','latex','FontSize',14);
    grid on