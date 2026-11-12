function sc = scenario(name)
%SCENARIO Named operating conditions for simulation and testing.
sc.name = name;  sc.T = 600;  sc.seed = 1;
sc.n_small = 30;  sc.n_border = 0;  sc.n_over = 0;
sc.current = [0 0];  sc.soc0 = 100;  sc.noise = 1.0;
sc.reject_enabled = true;        % false = baseline without gate/avoidance
sc.fail_time = inf;              % time at which health is lost [s]
switch name
    case 'calm'
    case 'mixed'
        sc.n_over = 6;  sc.n_border = 4;
    case 'mixed_no_reject'
        sc.n_over = 6;  sc.n_border = 4;  sc.reject_enabled = false;
    case 'current'
        sc.n_over = 4;  sc.current = [0.02 0.06];
    case 'noisy'
        sc.n_over = 6;  sc.n_border = 4;  sc.noise = 2.5;
    case 'low_battery'
        sc.soc0 = 20.3;  sc.T = 500;
    case 'dense'
        sc.n_small = 70;  sc.n_over = 4;  sc.T = 900;
    case 'link_loss'
        sc.fail_time = 120;  sc.T = 200;
    otherwise
        error('ankel:scenario', 'Unknown scenario %s', name);
end
end
