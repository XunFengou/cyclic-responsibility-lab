function plot_obstacle_comparison(root)
% Actual common-engine recordings; no cost-envelope substitution.
if nargin<1,root=fileparts(fileparts(mfilename('fullpath')));end
folder=fullfile(root,'research','obstacles');out=fullfile(root,'assets','result');
set(groot,'defaultFigureVisible','off');d=jsondecode(fileread(fullfile(folder,'three_obstacles.json')));m=jsondecode(fileread(fullfile(folder,'figure_map.json')));report=jsondecode(fileread(fullfile(out,'obstacle_benchmarks.json')));
f=figure('Color','w','Position',[50,50,1480,730]);tiledlayout(1,2,'TileSpacing','compact');theta=linspace(0,2*pi,400);
for j=1:2
 nexttile;hold on;if j==1,r=d.sector;else,r=d.voronoi;end
 if j==2
  [~,~,groups]=unique(sort(m.voronoiOwners,2),'rows');scatter(m.samples(:,1),m.samples(:,2),12,groups,'filled');colormap(gca,0.6+0.4*lines(max(groups)));
 else
  for a=m.theta',plot([0,cos(a)],[0,sin(a)],'-','Color',[.72,.75,.78],'LineWidth',.6);end
 end
 plot(cos(theta),sin(theta),'k-','LineWidth',1);
 for a=1:numel(r.paths),p=r.paths(a).points;plot(p(:,1),p(:,2),'-','Color',[.08,.42,.6],'LineWidth',.7);end
 for a=1:numel(d.config.obstacles),o=d.config.obstacles(a);fill(o.x+o.r*cos(theta),o.y+o.r*sin(theta),[.22,.29,.34],'EdgeColor','none');end
 p=reshape([r.agents.p],2,[])';plot(p(:,1),p(:,2),'o','MarkerSize',4,'MarkerFaceColor','w','Color',[.05,.3,.45]);
 axis equal;xlim([-1.08,1.08]);ylim([-1.08,1.08]);grid on;xlabel('Physical x');ylabel('Physical y');
 if j==1,title('Cyclic sectors: fixed owner order, obstacle-clipped atoms');else,title('Order-k Voronoi: colored nearest-k owner subsets');end
end
sgtitle(sprintf('Matched N=%d, k=%d, three obstacles, same initial positions; final t=24 s',d.config.N,d.config.k));finish(f,out,'M01_obstacle_responsibility_maps');
f=figure('Color','w','Position',[50,50,1480,1000]);tiledlayout(2,3,'TileSpacing','compact');fields={'cost','nearestCost','loadError','inputRms','clearance','cells'};ylabs={'Assigned H','Common k-nearest H','Relative RMS load error','RMS applied velocity','Minimum surface clearance','Sampled owner subsets'};
for j=1:6,nexttile;hold on;for a=1:2,if a==1,r=d.sector;else,r=d.voronoi;end;plot([r.history.time],[r.history.(fields{j})],'LineWidth',1.5);end;grid on;xlabel('Motion time (s)');ylabel(ylabs{j});title(ylabs{j});if j==1,legend({'Cyclic sectors','Order-k Voronoi'},'Location','best');end,end
sgtitle('Same environment and actuator/safety constraints: compare cost, loads, input, and topology');finish(f,out,'M02_matched_performance');
f=figure('Color','w','Position',[50,50,1500,650]);tiledlayout(1,3,'TileSpacing','compact');names={'failure_single','failure_complement','failure_switch'};labels={'Single order 7','Simultaneous 7 and 8','Persistent switching 7 / 8'};
for j=1:3,nexttile;hold on;for a=1:3,r=jsondecode(fileread(fullfile(folder,[names{a},'.json'])));h=r.sector.history;switch j,case 1,v=[h.massError];case 2,v=[h.blindError];case 3,v=[h.loadError];end;semilogy([h.time],max(v,1e-15),'LineWidth',1.5);end;xline(8,'k:');grid on;set(gca,'YScale','log');xlabel('Motion time (s)');switch j,case 1,ylabel('Relative basic-mass error');case 2,ylabel('Service-blind projection error');case 3,ylabel('Relative service-load error');end;if j==1,legend(labels,'Location','best');end;end
sgtitle('Same three-obstacle map, failure 50 to 49, service k=7; six blind modes appear');finish(f,out,'M03_fault_blind_modes');
fprintf('Completed three matched obstacle comparison figures from %d benchmark cases.\n',numel(report.cases));
end
function finish(f,out,name)
set(findall(f,'Type','axes'),'FontSize',11);exportgraphics(f,fullfile(out,[name,'.png']),'Resolution',150);close(f);
end
