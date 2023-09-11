
# Multivariable Adaptive Flight Control system for fixed-wing UAVs


The programs above are in three Matlab .m files, i.e.

- simulate_algorithm.m
- adaptive_algorithm.m
- simu_plot.m
⚠️ Save the A and B matrix of the UAV in `data.mat` as `A_LONG` and `B_LONG` respectively.\
\
In the Matlab environment, type the following command at the command window.
```
simulate_algorithm
```

The function ode45 in `simulate_algorithm.m `calls the function `adaptive_algorithm`
for each time step. \
\
The data generated by the programs is saved in the file `simulation_data.mat`
as the numerical solution to those differential equations to describe the dynamic system. \
\
Use `simu_plot` to plot the results of simulation. The simulation results are showed in the following
figures.