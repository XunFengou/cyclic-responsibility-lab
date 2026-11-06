function plot_tiago_comparison(root)
% Actual recorded MuJoCo and SI data. No envelopes or synthetic trajectories.
if nargin<1,root=fileparts(fileparts(mfilename('fullpath')));end
out=fullfile(root,'assets','result');d=jsondecode(fileread(fullfile(out,'tiago_benchmarks.json')));cases=d.cases;if iscell(cases),cases=[cases{:}];end
r=cases(2).frames;s=cases(3).frames;t=[r.time];blue=[.07 .42 .59];orange=[.75 .32 .13];set(groot,'defaultFigureVisible','off');
f=figure('Color','w','Position',[30 30 1650 1040]);tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
nexttile;semilogy(t,max(arrayfun(@(x)x.methods.single.loadError,r),1e-15),'-','Color',blue,'LineWidth',2);hold on;semilogy(t,max(arrayfun(@(x)x.methods.tiago.loadError,r),1e-15),'--','Color',orange,'LineWidth',1.4);grid on;xlabel('Motion time (s)');ylabel('Relative planned-load RMS');title('Shared partition law: curves overlap');legend({'Single integrator','TIAGo Pro EL'},'Location','southwest');
nexttile;plot(t,arrayfun(@(x)x.methods.single.cost,r),'Color',blue,'LineWidth',1.6);hold on;plot(t,arrayfun(@(x)x.methods.tiago.cost,r),'Color',orange,'LineWidth',1.6);grid on;xlabel('Motion time (s)');ylabel('Assigned H (m^2)');title('Actual distance cost; moving rays and contacts');legend({'Single integrator','TIAGo Pro EL'},'Location','northeast');
nexttile;plot(t,arrayfun(@(x)x.methods.single.inputRms,r),'Color',blue,'LineWidth',1.4);hold on;plot(t,arrayfun(@(x)x.methods.tiago.inputRms,r),'Color',orange,'LineWidth',1.4);grid on;xlabel('Motion time (s)');ylabel('Chassis speed RMS (m/s)');title('Common output: velocity input versus EL state');
nexttile;yyaxis left;plot(t,arrayfun(@(x)native(x,'baseForceRms'),r),'LineWidth',1.4);ylabel('EL chassis force RMS (N)');yyaxis right;plot(t,arrayfun(@(x)native(x,'armTorqueRms'),r),'LineWidth',1.4);ylabel('EL arm torque RMS (N m)');grid on;xlabel('Motion time (s)');title('Native force inputs; SI has no torque counterpart');
nexttile;plot(t,arrayfun(@(x)x.methods.single.realizedK,r),'-','Color',blue,'LineWidth',1.5);hold on;plot(t,arrayfun(@(x)x.methods.tiago.realizedK,r),'-','Color',orange,'LineWidth',1.5);plot(t,arrayfun(@(x)x.methods.single.diverseK,r),'--','Color',blue,'LineWidth',1.3);plot(t,arrayfun(@(x)x.methods.tiago.diverseK,r),'--','Color',orange,'LineWidth',1.3);grid on;xlabel('Motion time (s)');ylabel('Executed demand fraction');ylim([0 .6]);title('Allocation does not guarantee range / FOV / LOS');legend({'SI k-visible','EL k-visible','SI + view pair','EL + view pair'},'Location','southeast');
nexttile;semilogy(t,max(arrayfun(@(x)x.methods.single.massEnergy,r),1e-15),'Color',blue,'LineWidth',1.5);hold on;semilogy([s.time],max(arrayfun(@(x)x.methods.single.massEnergy,s),1e-15),'Color',orange,'LineWidth',1.5);semilogy([s.time],max(arrayfun(@(x)x.methods.single.blindEnergy,s),1e-15),'k--','LineWidth',1.2);grid on;xlabel('Motion time (s)');ylabel('Dimensionless coordination energy');title('gcd(6,2)=2: one blind mode; order 3 repairs it');legend({'Complement: E_{mass}','Single order: E_{mass}','Single order: E_{blind}'},'Location','southwest');
sgtitle({'PAL-derived 20-DoF TIAGo Pro vs single integrator: N=6, k=2, same 10 m obstacle domain','Official MuJoCo 3.8.1; planar support and ideal force actuators; numerical simulation, not hardware validation'},'FontWeight','bold');
exportgraphics(f,fullfile(out,'T01_TIAGo_matched_physics.png'),'Resolution',155);close(f);
end
function v=native(row,key)
v=row.native.(key);if isempty(v),v=NaN;end
end
