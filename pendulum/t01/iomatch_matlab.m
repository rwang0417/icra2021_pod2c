clear all;clc;tic;
%% system
SYS_NUM = 3;
IN_NUM = 1;
OUT_NUM=1;
STEP_NUM = 100;
TRIAL_NUM = 100;
SIM_STEP = 0.1;
CTRL_STEP = .1;
PERT_COEF = .1;
X_INIT = zeros(SYS_NUM,1);
% Bk=[0.4*eye(IN_NUM);0.2*ones(SYS_NUM-IN_NUM,IN_NUM)];
Bk=[1;2;3];
Ck=[1 0.2 0.4];%eye(SYS_NUM);%
Dk=0.0*ones(OUT_NUM,IN_NUM);
% Dk=[1;1];
%% read control sequence
% fid = fopen('result0.txt','r');
% U = fscanf(fid, '%f');
% fclose(fid);
% u_norm = reshape(U, IN_NUM, STEP_NUM);
% u_max = max(max(abs(u_norm)));
u_max=1;
u_norm=0.5*ones(IN_NUM,STEP_NUM);
%% nominal states
Ak=zeros(SYS_NUM,SYS_NUM,STEP_NUM);
Y_NORM = zeros(OUT_NUM,STEP_NUM+1);
x1 = X_INIT(:,1);
for i = 1 : 1 : STEP_NUM
    Ak(:,:,i)=[-0.2*cos((i-1)) 0 0;0 0.3*sin(3*(i-1)) 0;0 0 0.6];%expm([0 1;sin(3*(i-1))-1 -0.1]);%
    x2=Ak(:,:,i)*x1+Bk*u_norm(:,i);
    Y_NORM(:,i)=Ck*x1+Dk*u_norm(:,i);
    x1=x2;
end
Y_NORM(:,STEP_NUM+1)=Ck*x1;
%% collect data
delta_u=PERT_COEF*u_max*randn(IN_NUM*(STEP_NUM+1),TRIAL_NUM);
delta_y=zeros(OUT_NUM*(STEP_NUM+1),TRIAL_NUM);
for j=1:1:TRIAL_NUM
    x1 = X_INIT(:,1);
    for i=1:1:STEP_NUM
        x2=Ak(:,:,i)*x1+Bk*(u_norm(:,i)+delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j));
        delta_y(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+2),j)=Ck*x1+Dk*(u_norm(:,i)+delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j))-Y_NORM(:,i);
        x1=x2;
    end
    delta_y(1:OUT_NUM,j)=Ck*x1-Y_NORM(:,STEP_NUM+1);
end
%% id loop for each timestep
q=1;
qu=1;
fitcoef=zeros(OUT_NUM,OUT_NUM*q+IN_NUM*qu,STEP_NUM);
for i=max(q,qu)+2:1:STEP_NUM
    M1=[delta_y(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),:);delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+qu+1),:)];
%     M1=[delta_y(OUT_NUM*(STEP_NUM-i+q-1)+1:OUT_NUM*(STEP_NUM-i+1+q),:);delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+qu+1),:)];
    M3=[delta_y(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),:)];
    fitcoef(:,:,i)=delta_y(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)*M1'/(M1*M1');
    r(i)=sqrt(mean(mean((delta_y(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)-fitcoef(:,:,i)*M1).^2,1)));
    dtmt(:,:,i)=det(M1*M1');
end
%% prediction check with rolling window
pert_check=0.1;
ucheck=pert_check*u_max*randn(IN_NUM*(STEP_NUM+1),TRIAL_NUM);
y_sim=zeros(OUT_NUM*(STEP_NUM+1),TRIAL_NUM);
y_pred=zeros(OUT_NUM*(STEP_NUM+1),TRIAL_NUM);
for j=1:1:TRIAL_NUM
    x1 = X_INIT(:,1);
    for i=1:1:STEP_NUM
        x2=Ak(:,:,i)*x1+Bk*(u_norm(:,i)+ucheck(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j));
        y_sim(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+2),j)=Ck*x1+Dk*(u_norm(:,i)+ucheck(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j))-Y_NORM(:,i);
        x1=x2;
    end
    y_sim(1:OUT_NUM,j)=Ck*x1-Y_NORM(:,STEP_NUM+1);
end
y_pred(OUT_NUM*(STEP_NUM-q-1)+1:OUT_NUM*(STEP_NUM+1),:)=y_sim(OUT_NUM*(STEP_NUM-q-1)+1:OUT_NUM*(STEP_NUM+1),:);
for i=max(q,qu)+2:1:STEP_NUM
%     M2=[y_pred(OUT_NUM*(STEP_NUM-i+q-1)+1:OUT_NUM*(STEP_NUM-i+1+q),:);ucheck(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+qu+1),:)];%ucheck(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+qu+1),:)
    M2=[y_pred(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),:);ucheck(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+qu+1),:)];%ucheck(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+qu+1),:)
    y_pred(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)=fitcoef(:,:,i)*M2;
end
% compare
figure()
plot([fliplr(y_pred(10:end,1)')' fliplr(y_sim(10:end,1)')']);
legend('prediction','simulation');
%% LQR
A_aug(:,:,:)=[fitcoef(:,1:OUT_NUM*q,:),fitcoef(:,OUT_NUM*q+IN_NUM+1:end,:),zeros(OUT_NUM,IN_NUM,STEP_NUM);
              ones((q-1)*OUT_NUM,(q-1)*OUT_NUM,STEP_NUM),zeros((q-1)*OUT_NUM,OUT_NUM+IN_NUM*qu,STEP_NUM);
              zeros(IN_NUM*qu,OUT_NUM*q,STEP_NUM),[zeros(IN_NUM,IN_NUM*qu,STEP_NUM);ones(IN_NUM*(qu-1),IN_NUM*(qu-1),STEP_NUM),zeros(IN_NUM*(qu-1),IN_NUM,STEP_NUM)]];
B_aug(:,:,:)=[fitcoef(:,OUT_NUM*q+1:OUT_NUM*q+IN_NUM,:);zeros(OUT_NUM*(q-1),IN_NUM,STEP_NUM);ones(IN_NUM,IN_NUM,STEP_NUM);zeros(IN_NUM*(qu-1),IN_NUM,STEP_NUM)];
sig_q = 10^0;
sig_f = 10^5;
Ri = 10^0 * eye(IN_NUM);
Qi = sig_q * eye(OUT_NUM*q+IN_NUM*qu);Qi(OUT_NUM*q+1:end,OUT_NUM*q+1:end)=1*Qi(OUT_NUM*q+1:end,OUT_NUM*q+1:end);%Qi(1:OUT_NUM*q,1:OUT_NUM*q)=100*Qi(1:OUT_NUM*q,1:OUT_NUM*q);
OS = zeros(OUT_NUM*q+IN_NUM*qu, OUT_NUM*q+IN_NUM*qu, STEP_NUM+1);
TK = zeros(IN_NUM, OUT_NUM*q+IN_NUM*qu, STEP_NUM);
OS(:, :, STEP_NUM+1) = sig_f * eye(OUT_NUM*q+IN_NUM*qu);
for i= STEP_NUM:-1:1  
    OS(:, :, i) = A_aug(:, :, i)' * (OS(:, :, i +1) - OS(:, :, i + 1) * B_aug(:, :, i) / (B_aug(:, :, i)' * OS(:, :, i +1) * B_aug(:, :, i) + Ri) * B_aug(:, :, i)' * OS(:, :, i + 1)) * A_aug(:, :, i) + Qi;
end
for i = 1:1:STEP_NUM
    TK(:, :, i) = (Ri + B_aug(:, :, i)' * OS(:, :, i+1) * B_aug(:, :, i)) \ B_aug(:, :, i)' * OS(:, :, i+1) * A_aug(:, :, i);
%     TK(:, :, i)=dlqr(A_aug(:, :, i),B_aug(:, :, i),Qi,Ri);
end
% performance check
TEST_NUM=100;
pert_test=1;
delta_u_test=[pert_test*u_max*randn(IN_NUM*(STEP_NUM+1),TEST_NUM);zeros(IN_NUM*(qu+1),TEST_NUM)];
y_open=zeros(OUT_NUM*(STEP_NUM+1),TEST_NUM);
y_closed=zeros(OUT_NUM*(STEP_NUM+1),TEST_NUM);
for j=1:1:TEST_NUM
    % openloop
    x1 = X_INIT(:,1);
    for i=max(q,qu)+2:1:STEP_NUM
        x2=Ak(:,:,i)*x1+Bk*(u_norm(:,i)+delta_u_test(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j));
        y_open(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+2),j)=Ck*x1+Dk*(u_norm(:,i)+delta_u_test(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j))-Y_NORM(:,i);
        x1=x2;
    end
    y_open(1:OUT_NUM,j)=Ck*x1-Y_NORM(:,STEP_NUM+1);
    x1 = X_INIT(:,1);u_feedback=0;
    % closedloop correct when Dk=0
    for i=max(q,qu)+2:1:STEP_NUM
        y_closed(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+2),j)=Ck*x1+Dk*(u_norm(:,i)+delta_u_test(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j))-Y_NORM(:,i);
        u_feedback=-TK(:,:,i)*[y_closed(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),j);delta_u_test(IN_NUM*(STEP_NUM-i+2)+1:IN_NUM*(STEP_NUM-i+2+qu),j)];
        x2=Ak(:,:,i)*x1+Bk*(u_norm(:,i)+delta_u_test(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j)+u_feedback);
        x1=x2;
    end
    y_closed(1:OUT_NUM,j)=Ck*x1-Y_NORM(:,STEP_NUM+1);
    for i=3:1:100
        yy(:,i)=Ck*x1;
        u_feedback=-TK(:,:,30)*[yy(:,i);zeros(IN_NUM*qu,1)];
        x2=Ak(:,:,i)*x1+Bk*(u_feedback);
        x1=x2;
    end
end
y_closed_avg=mean(abs(y_closed),2);
y_open_avg=mean(abs(y_open),2);
figure()
plot([fliplr(y_open_avg(1:end)')' fliplr(y_closed_avg(1:end)')']);
legend('openloop','closedloop');
openloop=mean(y_open_avg)
closedloop=mean(y_closed_avg)
figure()
plot(yy(1,:))
