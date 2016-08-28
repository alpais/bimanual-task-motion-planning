%% Save data from datafile 

% for i = 1:10 
%     datapos = [E2_complet.S2.EEPOS.X(i, 1:3000)' E2_complet.S2.EEPOS.Y(i, 1:3000)' E2_complet.S2.EEPOS.Z(i, 1:3000)']; 
%     sname = sprintf('s1_pos_data_%0.3d.txt',i);
%     save(sname,'datapos','-ascii');
% end

%% Load Data

% Initialize variables
clear
figure(1); cla;  %axis([-0.2000 0 -0.2000 0 0 0.1400]);
hold on; grid on; view(3);
figure(2); clf; hold on;
figure(3); clf; hold on;

pdata={};
odata={};
odata_dyn={};
count=1;
otarget = zeros(4,1);

%Indexes of demonstrations to be used in learning
file_ind = [1 2 3 4 5];
% file_ind = [1 6 12 13];
% file_ind = [1 3 4 ];

%Loading data
for i=1:length(file_ind)
   x=load(sprintf('data_master_arm/data_pos_%0.3d.txt',file_ind(i))); 
  % x = x(1:10:end-50,:); %downsampling by 10
   for j=1:size(x,2)
      x(:,j)=smooth(x(:,j),100); 
   end

%    x(:,7) = 0;
   otarget = otarget + x(end,4:7)';      % Setting the target of the orientation as the last point of the demonstrations
   odata{count} = x(:,4:7)';             % Separating orientation data   
   disp(x(end,:));                       % The last element of x   
   x = x - repmat(x(end,:),size(x,1),1); % Substracting the last value from all elements of x
                                         % Now the values in x should decrease nicely to zero. Note that SEDS will
                                         % add a (0, 0, 0) to the end of each demonstration (which corresponds to
                                         % zero velocity at the end of each demo), but if it doesn't decrease nicely
                                         % it will affect the dynamics by being a too sudden change. Can cause
                                         % overshooting the attractor. It happens frequently in learning from
                                         % segmented data if not every part of the demonstration ended with zero
                                         % velocity.
   % Plot demonstration data - 3D Position
   figure(1)
   plot3(x(1:10:end,1),x(1:10:end,2),x(1:10:end,3),'-.r');
   xlabel('X Position');
   ylabel('Y Position');
   zlabel('Z Position');
   view(-62,22);
   
   figure(2)
   nrmx=sqrt(sum(x(:,1:3).^2,2));
   subplot(3,1,1); hold on; plot(nrmx);         ylabel('Norm of position');
   subplot(3,1,2); hold on; plot(x(:,4:7),'.'); ylabel('Orient dyn');
   subplot(3,1,3); hold on; plot(x(:,1:3),'.'); ylabel('Pos dyn');
   pdata{count} = x(:,1:3)';             % Should be called pdata_dyn
   odata_dyn{count} = x(:,4:7)';
   count=count+1;
%    pause
end
otarget = otarget/length(file_ind);      % Takes the average target from all the demonstrations
figure(1)
axis equal 
axis tight

%% Coupling model

cplfnc = @(x)([sum(x(1:2,:).^2,1);x(3,:)]); % 2D coupling function ==> norm in x-y axes combined with z coordinate
                                            % Symbolic function for input argument x, returns a 2D vector, 
                                            % representing the norm in the first 2 dimensions of x and 
                                            % the 3rd dimension of x. The dimension of the returned vector is the
                                            % output dimension of the cplGMM

cpldata=[];
for i=1:length(odata)                           % Actually the number of demonstrations stored as cells in odata
    cpldata=[cpldata,[pdata{i};odata_dyn{i}]];  % Concatenates demonstration data (matrix 7 rows - pos, or)
end                                             % Takes relative data, so model will also be relative

% Learns a mapping between the output of the coupling function (a 2D
% vector) and the orientation values (rows 4:7 of cpldata)
cplGMM = learn_coupling(cpldata, ...            % Structure of learned coupling, Data
                        1:3, ...                % master dimension
                        4:7, ...                % master dimension
                        cplfnc, ...             % coupling function
                        8);                     % number of gaussians


ndem = 4;                                               % ==> Demonstration 6 for testing
x=load(sprintf('data_master_arm/data_pos_%0.3d.txt', ndem));         % Loading a demo for testing the coupling model. 
                                                        % This overwrites the original x, initialized above
x(:,1:3) = x(:,1:3) - repmat(x(end,1:3),size(x,1),1);   % Substracting attractor
 
% Calculating orientation based on the coupling and 
% giving as input the position vectors in the loaded demonstration.
or = GMR(cplGMM.Priors, cplGMM.Mu, cplGMM.Sigma, ...    % cpl GMM
     cplfnc(x(:,1:3)'), ...                             % computes the value of the coupling function for the loaded demo
     1:cplGMM.coupl_dim, ...                            % input dimension (is the dimension returned by the cplfnc)
     cplGMM.coupl_dim+1:size(cplGMM.Mu,1));             % output dimension (in this case 4, the orientation dim)

figure(3); 
subplot(2,1,1); hold on; plot(x(:,1:3));                % Plotting position from demo
subplot(2,1,2); hold on; plot(or','-','Linewidth',1);   % Plotting corresponding obtained orientation

% ====== SAVING ======= 
structGMM = cplGMM; 
save('master_cplGMM.mat','structGMM');
saveGMM('master_cplGMM.mat','master_cplGMM.txt');

%% Master dyn ==> SEDS

[x0 , xT, Data, index] = preprocess_demos(pdata, ... % Takes the substracted position data. Computes first derivative and cuts traj
                        0.001*10, ...                % Sampling rate. We multiply with the decimation factor
                        0.02);                        % tolerance for cutting. If the derivative is below this value data is deleted
                                                               

K = 1; % Number of Gaussian functions

% A set of options that will be passed to the solver. Please type 'doc
% preprocess_demos' in the MATLAB command window to get detailed
% information about each option.
options=[];
options.tol_mat_bias = 10^-6;   % to avoid instabilities in the gaussian kernel
options.perior_opt = 1;
options.mu_opt = 1;             % optimize centers
options.sigma_x_opt = 1;
options.display = 1;
options.tol_stopping=10^2;
options.max_iter =5000;
options.normalization = 1;
options.objective = 'likelihood';
options.cons_penalty=1e10;      % penalty for not going straight to the attractor. 
                                % Increase to obtain a more straight line


[Priors, Mu, Sigma] = initialize_SEDS(Data, K);                      % finding an initial guess for GMM's parameter
[Priors Mu Sigma]   = SEDS_Solver(Priors, Mu, Sigma, Data, options); % running SEDS optimization solver

masterGMM = [];
masterGMM.Priors = Priors;
masterGMM.Mu = Mu;
masterGMM.Sigma = Sigma;
masterGMM.States = K;

fn_handle = @(y) GMR(Priors, Mu, Sigma, y, 1:3, 4:6);               % Symbolic function for performing GMR on data. Input dimension 1:3
                                                                    % represents positions and output dimension 4:6 returns velocities
x0_all = Data(1:3,index(1:end-1)) + 0.0*randn(3,length(index)-1);   % Taking initial positions of each demo and adding a small displacement
opt_sim.dt = 0.01;                                                  % Should be the same as the learning dt        
opt_sim.i_max = 3000;                                               % max number of iterations    
opt_sim.tol = 0.001;                                                % reaching tolerance
[x, xd, tmp, xT] = Simulation(x0_all,[],fn_handle,opt_sim);         %running the simulator

% ====== SAVING ======= 
structGMM = masterGMM;    
save('master_posGMM.mat','structGMM');
saveGMM('master_posGMM.mat','master_posGMM.txt');

%% Slave dyn ==> SEDS

[x0 , xT, Data, index] = preprocess_demos(odata_dyn,0.001*10,0.0);

K =1; %Number of Gaussian functions

% A set of options that will be passed to the solver. Please type 'doc
% preprocess_demos' in the MATLAB command window to get detailed
% information about each option.
options.tol_mat_bias = 10^-6;
options.perior_opt = 1;
options.mu_opt =1;
options.sigma_x_endopt = 1;
options.display = 1;
options.tol_stopping=10^-10;
options.max_iter =5000;
options.normalization = 1;
options.objective = 'likelihood';
options.cons_penalty=1e10;

[Priors, Mu, Sigma] = initialize_SEDS(Data,K); %finding an initial guess for GMM's parameter
[Priors Mu Sigma]   = SEDS_Solver(Priors,Mu,Sigma,Data,options); %running SEDS optimization solver

masterGMM = [];
masterGMM.Priors = Priors;
masterGMM.Mu = Mu;
masterGMM.Sigma = Sigma;
masterGMM.States = K;

% Testing the obtained model
fn_handle = @(y) GMR(Priors,Mu,Sigma,y,1:4,5:8); % in - orientation; out - orient velocity
x0_all        = Data(1:4,index(1:end-1));
opt_sim.dt    = 0.02;
opt_sim.i_max = 3000;
opt_sim.tol   = 0.001;
[x, xd, tmp, xT] = Simulation(x0_all,[],fn_handle,opt_sim); %running the simulator; Overwrittes x again

figure(4); cla;
subplot(1,2,1); hold on; plot(x(:,:,2)'); 
subplot(1,2,2); hold on; plot(odata_dyn{1}'); plot(odata_dyn{2}'); plot(odata_dyn{3}');

figure(5); cla; hold on;
axis([-1 1 -1 1 -1 1]);
for i = 1:10:size(x,2)
    disp(i);
    PlotAxis([0;0;0], x(:,i,1) + otarget, 'aa',0.5);
    pause(0.01);
end

% ====== SAVING ======= 
structGMM = masterGMM;
save('master_oriGMM.mat','structGMM'); 
saveGMM('master_oriGMM.mat','master_oriGMM.txt');

%% Simulation using coupling
dt = 0.01;
load master_cplGMM;    cplGMM = structGMM;     cplfnc = cplGMM.cplfunc;
load master_posGMM; masterGMM = structGMM;
load master_oriGMM;  masterGMM = structGMM;

figure('Color',[1 1 1]); 
%axis([-0.2000         0   -0.2000         0         0    0.1400]);
hold on; grid on; view(-62,22);
xlabel('X Position');
ylabel('Y Position');
zlabel('Z Position');
pause;

%subplot(1,2,1); cla; view(3);  axis equal
%subplot(1,2,2); cla; hold on; grid on; set(gca,'XLim',[0 200]);
% set(gca,'XLim',[min(Data(1,:)),max(Data(1,:))]);
% set(gca,'YLim',[min(Data(2,:)),max(Data(2,:))]);
% set(gca,'ZLim',[min(Data(3,:)),max(Data(3,:))]);

% Starting from one of the demonstrations
demo_num = 2;
pos = pdata{demo_num}(:,1);         % Starting position - these are already substracted i.e. relative
                                    % Pose is in attractor frame
or  = odata{demo_num}(:,1);         % Starting orientation - not relative (not substracted)
otarget = odata{demo_num}(:,end);   % Target - from initial data
                                    
%subplot(1,2,1);
h  = PlotAxis(pos(:,end), or, 'aa', 0.1);
sp = plot3(pos(1,:)',pos(2,:)',pos(3,:)','-','Erasemode','none');
t=0;

pvel = GMR(masterGMM.Priors, masterGMM.Mu, masterGMM.Sigma, pos, 1:3, 4:6); % Computing initial pos velocity based on model
while(norm(pvel) > 1e-2)   
    pos = [pos, pos(:,end) + dt*pvel];                      % Compute the next desired position by integrating velocity - Simulate position        
    dor = GMR(cplGMM.Priors, cplGMM.Mu, cplGMM.Sigma, ...   % Compute the desired orientation based on coupling
            cplfnc(pos(:,end)), 1:cplGMM.coupl_dim, ...
            cplGMM.coupl_dim+1:size(cplGMM.Mu,1)) + otarget; % Add the target orientation
    
% For Simple (linear) master dynamics. Replace this with CDDynamics or other
% eor = (dor-or(:,end))*50; or=[or, or(:,end) + eor*dt];

    %GMR
    eor = GMR(masterGMM.Priors, masterGMM.Mu, masterGMM.Sigma, or(:,end) - dor, 1:4, 5:8); % Compute orientation error
    or=[or, or(:,end) + 1*eor*dt];                                                      % Compute next orientation by integrating
    
% Through
%     or = [or, dor];
    
    subplot(1,2,1);
    set(sp, 'XData',pos(1,:),'YData',pos(2,:),'ZData',pos(3,:));

    delete(h);
    h=PlotAxis(pos(:,end), or(:,end), 'aa', 0.05);
    subplot(1,2,2); plot(or'); plot(t,dor,'.');
    t=t+1;
    
    pvel = GMR(masterGMM.Priors, masterGMM.Mu, masterGMM.Sigma, pos(:,end), 1:3, 4:6); % Compute next velocity
    pause(0.001);
    %axis tight
    
end






