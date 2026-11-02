function plot_research(root)
% Reproduce paper figures from the actual website engine; MATLAB R2023b.
if nargin<1,root=fileparts(fileparts(mfilename('fullpath')));end
source=fullfile(root,'research');out=fullfile(root,'assets','result');if ~exist(out,'dir'),mkdir(out);end
set(groot,'defaultFigureVisible','off');names={'single','double','high','unicycle','differential','lagrangian'};data=cell(1,6);labels=cell(1,6);
for i=1:6,data{i}=readcase(source,['fixed_',names{i}]);labels{i}=data{i}.dynamicsMetadata.name;end
f=figure('Color','w','Position',[50,50,1480,980]);tiledlayout(2,3,'TileSpacing','compact');
for i=1:6
 d=data{i};nexttile;hold on;t=linspace(0,2*pi,300);plot(cos(t),sin(t),'k-');
 for a=1:numel(d.trajectories),p=d.trajectories(a).points;plot(p(:,1),p(:,2),'-','Color',[.10,.48,.67],'LineWidth',.7);end
 p=d.positions;c=d.final.centroids;plot(p(:,1),p(:,2),'o','Color',[.06,.32,.5],'MarkerFaceColor','w','MarkerSize',4);plot(c(:,1),c(:,2),'+','Color',[.75,.35,.1]);axis equal;xlim([-1.1,1.1]);ylim([-1.1,1.1]);grid on;xlabel('Physical x');ylabel('Physical y');title(labels{i});
end
sgtitle('Matched initial positions, fixed partition, service k=5: actual robot paths');finish(f,out,'D01_actual_dynamics_paths');
f=figure('Color','w','Position',[50,50,1480,980]);tiledlayout(2,3,'TileSpacing','compact');
for i=1:6
 d=data{i};nexttile;plot([d.history.motionTime],[d.history.costPerOrder],'LineWidth',1.5);hold on;plot([d.history.motionTime],[d.history.centroidCostLowerBound],'--','LineWidth',1.2);grid on;xlabel('Motion time (s)');ylabel('Actual physical cost J / k');title(labels{i});legend({'Recorded cost','Centroid lower bound'},'Location','best');
end
sgtitle('Fixed-partition cost descent: no decreasing envelope is substituted');finish(f,out,'D02_physical_cost_descent');
f=figure('Color','w','Position',[50,50,1530,1000]);tiledlayout(2,3,'TileSpacing','compact');
for i=1:6
 d=data{i};nexttile;s=d.inputSeries(1).samples;t=[s.motionTime];u=reshape([s.input],2,[])';yyaxis left;plot(t,u(:,1),'LineWidth',1.3);ylabel(d.dynamicsMetadata.units{1});yyaxis right;plot(t,u(:,2),'LineWidth',1.3);ylabel(d.dynamicsMetadata.units{2});grid on;xlabel('Motion time (s)');title(labels{i});legend(d.dynamicsMetadata.channels,'Location','best');
end
sgtitle('Actual native actuator inputs for physical robot ID 1; different units across models');finish(f,out,'D03_native_control_inputs');
f=figure('Color','w','Position',[50,50,1500,620]);tiledlayout(1,3,'TileSpacing','compact');
for j=1:3,nexttile;hold on;for i=1:6,d=readcase(source,['fault_',names{i}]);t=[d.history.motionTime];switch j,case 1,semilogy(t,max([d.history.massError],1e-15),'LineWidth',1.2);case 2,semilogy(t,max([d.history.targetError],1e-10),'LineWidth',1.2);case 3,plot(t,[d.history.costPerOrder],'LineWidth',1.2);end,end
 xline(40,'k:');grid on;xlabel('Motion time (s)');if j<3,set(gca,'YScale','log');end
 switch j,case 1,ylabel('Relative basic-mass error');title('Same cyclic operator: six mass curves coincide');case 2,ylabel('Maximum feasible-target error');title('Different plant transients and goal tolerances');case 3,ylabel('Physical service cost J / k');title('Actual coupled cost includes a failure jump');end
 if j==2,legend(labels,'Location','best');end
end
finish(f,out,'D04_algebra_vs_robot_dynamics');
status=struct('status','completed','matlab',version,'figures',4,'source','Recorded browser-engine equations, seed 2718','modelChoices',6);fid=fopen(fullfile(out,'dynamics_plot_status.json'),'w');fprintf(fid,'%s',jsonencode(status,'PrettyPrint',true));fclose(fid);
fprintf('Completed four dynamics paper figures.\n');
end
function d=readcase(folder,name),d=jsondecode(fileread(fullfile(folder,[name,'.json'])));end
function finish(f,out,name),set(findall(f,'Type','axes'),'FontSize',10);exportgraphics(f,fullfile(out,[name,'.png']),'Resolution',150);savefig(f,fullfile(fileparts(fileparts(out)),'research',[name,'.fig']));close(f);end
