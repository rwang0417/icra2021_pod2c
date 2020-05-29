clear all;clc;tic;
%% pendulum
POS_NUM = 1;
VEL_NUM = 1;
SYS_NUM = 2;
IN_NUM = 1;
OUT_NUM= 2;
MODEL = 'pendulum.xml';
STEP_NUM = 200;
TRIAL_NUM = 50;
PERT_COEF = .2;
X_INIT = [pi;0];
X_TARGET = [0;0];
Ck=eye(SYS_NUM);%[1 0];%[0 1];%
Dk=0.0*ones(OUT_NUM,IN_NUM);

%% cartpole
% POS_NUM = 2;
% VEL_NUM = 2;
% SYS_NUM = 4;
% IN_NUM = 1;
% OUT_NUM= 2;
% MODEL = 'cartpole.xml';
% STEP_NUM = 30;
% TRIAL_NUM = 600;
% PERT_COEF = .2;
% X_INIT = zeros(SYS_NUM,1);
% Ck=[1 0 0 0;0 1 0 0];%eye(SYS_NUM);%[1 0 0 0];%[0 0 1 0;0 0 0 1];%[0.3 0.4 0.5 0.6];%[0.5 1 0.2 0.4; 0 0.2 0.5 0.9;0 0.3 0.7 0.2;0.2 0.3 0.1 0.4];%
% Dk=0.0*ones(OUT_NUM,IN_NUM);

%% swimmer3
% POS_NUM = 5;
% VEL_NUM = 5;
% SYS_NUM = 10;
% IN_NUM = 2;
% OUT_NUM= 5;
% MODEL = 'swimmer3.xml';
% STEP_NUM = 1600;
% TRIAL_NUM = 400;
% PERT_COEF = .01;
% X_INIT = zeros(SYS_NUM,1);
% Ck=[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM) ];%eye(SYS_NUM);%[zeros(POS_NUM,POS_NUM) eye(VEL_NUM)];%
% Dk=0.0*ones(OUT_NUM,IN_NUM);

%% swimmer6
% POS_NUM = 8;
% VEL_NUM = 8;
% SYS_NUM = 16;
% IN_NUM = 5;
% OUT_NUM= 16;
% MODEL = 'swimmer6.xml';
% STEP_NUM = 1500;
% TRIAL_NUM = 400;
% PERT_COEF = .01;
% X_INIT = zeros(SYS_NUM,1);
% Ck=eye(SYS_NUM);%[zeros(POS_NUM,POS_NUM) eye(VEL_NUM)];%[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM) ];%
% Dk=0.0*ones(OUT_NUM,IN_NUM);

%% tuning
% fitting
q=1;
qu=1;
% lqr
sig_q = 10^2;
sig_f = 10^4;
sig_r = 10^0;
X_INIT_TEST(:,1)=X_INIT(:,1);%[0.;0.;];%

%% variables
u_norm = zeros(IN_NUM,STEP_NUM);
delta_y = zeros(OUT_NUM*(STEP_NUM+1),TRIAL_NUM);
Y_NORM = zeros(OUT_NUM,STEP_NUM+1);
fitcoef = zeros(OUT_NUM,OUT_NUM*q+IN_NUM*qu,STEP_NUM);
A_aug = zeros(OUT_NUM*q+IN_NUM*qu,OUT_NUM*q+IN_NUM*qu,STEP_NUM);
B_aug = zeros(OUT_NUM*q+IN_NUM*qu,IN_NUM,STEP_NUM);
Ri = sig_r *1* eye(IN_NUM);
Qi = sig_q * eye(OUT_NUM*q+IN_NUM*qu);Qi(OUT_NUM+1:end,OUT_NUM+1:end)=1/sig_q*0*Qi(OUT_NUM+1:end,OUT_NUM+1:end);%Qi(OUT_NUM*q+1:end,OUT_NUM*q+1:end)=1*Qi(OUT_NUM*q+1:end,OUT_NUM*q+1:end);%Qi(1:OUT_NUM*q,1:OUT_NUM*q)=100*Qi(1:OUT_NUM*q,1:OUT_NUM*q);
Kx = zeros(IN_NUM,OUT_NUM*q+IN_NUM*qu,STEP_NUM);
Z_NORM = zeros(OUT_NUM*(STEP_NUM+1),1);
Z_NORM_PREV = zeros(OUT_NUM*(STEP_NUM+1),1);
U_RCD = zeros(IN_NUM*(STEP_NUM+1),1);
U_RCD_PREV = zeros(IN_NUM*(STEP_NUM+1),1);

for ite=1:1:20
%% nominal trajectory
mexstep('load',MODEL); % load model
mexstep('reset');
mexstep('set','qpos',X_INIT(1:POS_NUM,1),POS_NUM);
mexstep('set','qvel',X_INIT(POS_NUM+1:SYS_NUM,1),VEL_NUM);
mexstep('forward');
Y_NORM(:,1)=Ck*X_INIT(:,1);
Z_NORM(OUT_NUM*STEP_NUM+1:OUT_NUM*(STEP_NUM+1),1) = Ck*X_INIT(:,1);
for i = 1 : 1 : STEP_NUM
    if i >= max(q,qu)+2
        u_norm(:,i) = u_norm(:,i)-Kx(:,:,i)*[(Z_NORM(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),1)-Z_NORM_PREV(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),1));(U_RCD(IN_NUM*(STEP_NUM-i+2)+1:IN_NUM*(STEP_NUM-i+2+qu),1)-U_RCD_PREV(IN_NUM*(STEP_NUM-i+2)+1:IN_NUM*(STEP_NUM-i+2+qu),1))];
    end
    U_RCD(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),1) = u_norm(:,i);
    mexstep('set','ctrl',u_norm(:,i),IN_NUM);
    mexstep('step',1);
    x2(1:POS_NUM,1)=mexstep('get','qpos');
    x2(POS_NUM+1:SYS_NUM,1)=mexstep('get','qvel');
    Y_NORM(:,i+1)=Ck*x2;
    Z_NORM(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),1) = Ck*x2;
end
Z_NORM_PREV = Z_NORM;
U_RCD_PREV = U_RCD;

%% collect input-output data for backward pass
delta_u=PERT_COEF*1*randn(IN_NUM*(STEP_NUM+1),TRIAL_NUM);
for j=1:1:TRIAL_NUM
    mexstep('reset');
    mexstep('set','qpos',X_INIT(1:POS_NUM,1),POS_NUM);
    mexstep('set','qvel',X_INIT(POS_NUM+1:SYS_NUM,1),VEL_NUM);
    mexstep('forward');
    for i=1:1:STEP_NUM
        mexstep('set','ctrl',u_norm(:,i)+delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j),IN_NUM);
        mexstep('step',1);
        x2(1:POS_NUM,1)=mexstep('get','qpos');
        x2(POS_NUM+1:SYS_NUM,1)=mexstep('get','qvel');
        delta_y(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),j)=Ck*x2-Y_NORM(:,i+1);
    end
end 

%% least square fitting
for i=max(q,qu)+2:1:STEP_NUM % 加一因为是zero init
    M1=[delta_y(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),:);delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+qu+1),:)];
%     M1=[delta_y(OUT_NUM*(STEP_NUM-i+q-1)+1:OUT_NUM*(STEP_NUM-i+1+q),:);delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+qu+1),:)];
%     M3=[delta_y(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),:)];
    fitcoef(:,:,i)=delta_y(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)*M1'/(M1*M1');
%     r(i)=sqrt(mean(mean((delta_y(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)-fitcoef(:,:,i)*M1).^2,1)));
%     dtmt(:,:,i)=det(M1*M1');
end

%% backpropagation
for i=max(q,qu)+2:1:STEP_NUM
A_aug(:,:,i)=[fitcoef(:,1:OUT_NUM*q,i),fitcoef(:,OUT_NUM*q+IN_NUM+1:end,i),zeros(OUT_NUM,IN_NUM);
              eye((q-1)*OUT_NUM),zeros((q-1)*OUT_NUM,OUT_NUM+IN_NUM*qu);
              zeros(IN_NUM*qu,OUT_NUM*q),[zeros(IN_NUM,IN_NUM*qu);eye(IN_NUM*(qu-1)),zeros(IN_NUM*(qu-1),IN_NUM)]];
B_aug(:,:,i)=[fitcoef(:,OUT_NUM*q+1:OUT_NUM*q+IN_NUM,i);zeros(OUT_NUM*(q-1),IN_NUM);eye(IN_NUM);zeros(IN_NUM*(qu-1),IN_NUM)];
end
Vxx = zeros(OUT_NUM*q+IN_NUM*qu, OUT_NUM*q+IN_NUM*qu, STEP_NUM+1);
Vxx(:, :, STEP_NUM+1) = sig_f * eye(OUT_NUM*q+IN_NUM*qu);Vxx(OUT_NUM+1:end,OUT_NUM+1:end)=1/sig_f*0*Vxx(OUT_NUM+1:end,OUT_NUM+1:end);
Vx = zeros(OUT_NUM*q+IN_NUM*qu, 1, STEP_NUM+1);
Vx(:, :, STEP_NUM+1) = Vxx(:, :, STEP_NUM+1)*[(Y_NORM(:,STEP_NUM+1)-Ck*X_TARGET);zeros(OUT_NUM*(q-1)+IN_NUM*qu,1)];
for i= STEP_NUM:-1:max(q,qu)+2  
    ITM=(B_aug(:, :, i)'*Vxx(:,:,i+1)*B_aug(:, :, i)+Ri);
    Kx(:,:,i)=ITM\B_aug(:, :, i)'*Vxx(:,:,i+1)*A_aug(:, :, i);
    Kv=ITM\B_aug(:, :, i)';
    Ku=ITM\Ri;
    Vxx(:,:,i)=A_aug(:,:,i)'*Vxx(:,:,i+1)*(A_aug(:,:,i)-B_aug(:, :, i)*Kx(:,:,i)) + Qi;
    Vx(:,:,i)=(A_aug(:,:,i)-B_aug(:, :, i)*Kx(:,:,i))'*Vx(:,:,i+1)-Kx(:,:,i)'*Ri*u_norm(:,i)+Qi*[(Y_NORM(:,i)-Ck*X_TARGET);zeros(OUT_NUM*(q-1)+IN_NUM*qu,1)];
    u_norm(:,i) = u_norm(:,i)-Kv*Vx(:,:,i+1)-Ku*u_norm(:,i);
end
end
mexstep('exit');
toc;

% %% LQR
% for i=max(q,qu)+2:1:STEP_NUM
% A_aug(:,:,i)=[fitcoef(:,1:OUT_NUM*q,i),fitcoef(:,OUT_NUM*q+IN_NUM+1:end,i),zeros(OUT_NUM,IN_NUM);
%               eye((q-1)*OUT_NUM),zeros((q-1)*OUT_NUM,OUT_NUM+IN_NUM*qu);
%               zeros(IN_NUM*qu,OUT_NUM*q),[zeros(IN_NUM,IN_NUM*qu);eye(IN_NUM*(qu-1)),zeros(IN_NUM*(qu-1),IN_NUM)]];
% B_aug(:,:,i)=[fitcoef(:,OUT_NUM*q+1:OUT_NUM*q+IN_NUM,i);zeros(OUT_NUM*(q-1),IN_NUM);eye(IN_NUM);zeros(IN_NUM*(qu-1),IN_NUM)];
% end
% Ri = sig_r *1* eye(IN_NUM);
% Qi = sig_q * eye(OUT_NUM*q+IN_NUM*qu);Qi(OUT_NUM+1:end,OUT_NUM+1:end)=1/sig_q*0*Qi(OUT_NUM+1:end,OUT_NUM+1:end);%Qi(OUT_NUM*q+1:end,OUT_NUM*q+1:end)=1*Qi(OUT_NUM*q+1:end,OUT_NUM*q+1:end);%Qi(1:OUT_NUM*q,1:OUT_NUM*q)=100*Qi(1:OUT_NUM*q,1:OUT_NUM*q);
% OS = zeros(OUT_NUM*q+IN_NUM*qu, OUT_NUM*q+IN_NUM*qu, STEP_NUM+1);
% TK = zeros(IN_NUM, OUT_NUM*q+IN_NUM*qu, STEP_NUM);
% OS(:, :, STEP_NUM+1) = sig_f * eye(OUT_NUM*q+IN_NUM*qu);OS(OUT_NUM+1:end,OUT_NUM+1:end)=1/sig_f*0*OS(OUT_NUM+1:end,OUT_NUM+1:end);
% for i= STEP_NUM:-1:max(q,qu)+2  
%     OS(:, :, i) = A_aug(:, :, i)' * (OS(:, :, i +1) - OS(:, :, i + 1) * B_aug(:, :, i) / (B_aug(:, :, i)' * OS(:, :, i +1) * B_aug(:, :, i) + Ri) * B_aug(:, :, i)' * OS(:, :, i + 1)) * A_aug(:, :, i) + Qi;
% end
% for i = max(q,qu)+2:1:STEP_NUM
%     TK(:, :, i) = (Ri + B_aug(:, :, i)' * OS(:, :, i+1) * B_aug(:, :, i)) \ B_aug(:, :, i)' * OS(:, :, i+1) * A_aug(:, :, i);
% %     TK(:, :, i)=dlqr(A_aug(:, :, i),B_aug(:, :, i),Qi,Ri);
% end
% 
% % % print TK to file
% % fidtk = fopen('TKaug.txt','wt');
% % for k = 1:1:STEP_NUM
% %     for i = 1:1:IN_NUM
% %         fprintf(fidtk,'%f ',TK(i,:,k));
% %         fprintf(fidtk,'\n');
% %     end
% %     fprintf(fidtk,'\n');
% % end
% % fclose(fidtk);
% 
% MTK=TK;
% MCK=Ck;
% MQ=q;
% MQU=qu;
% save('feedbackioid.mat','MTK','MCK','MQ','MQU');
% 
% % performance check
% pert_test=.1;
% STEP_NUM_SHOW=STEP_NUM;
% delta_u_test=[pert_test*u_max*randn(IN_NUM*(STEP_NUM_SHOW+1),TEST_NUM);zeros(IN_NUM*(qu+1),TEST_NUM)];
% y_open=zeros(OUT_NUM*(STEP_NUM_SHOW+1),TEST_NUM);
% y_closed=zeros(OUT_NUM*(STEP_NUM_SHOW+1),TEST_NUM);
% u_feedback=zeros(IN_NUM*(STEP_NUM_SHOW+1),TEST_NUM);
% for j=1:1:TEST_NUM
%     % openloop
%     mexstep('reset');
%     mexstep('set','qpos',X_INIT_TEST(1:POS_NUM,1),POS_NUM);
%     mexstep('set','qvel',X_INIT_TEST(POS_NUM+1:SYS_NUM,1),VEL_NUM);
%     mexstep('forward');
%     for i=1:1:STEP_NUM
%         if i >= max(q,qu)+2
%             mexstep('set','ctrl',u_norm(:,i)+delta_u_test(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j),IN_NUM);
%         else
%             mexstep('set','ctrl',u_norm(:,i),IN_NUM);
%         end
%         mexstep('step',1);
%         x2(1:POS_NUM,1)=mexstep('get','qpos');
%         x2(POS_NUM+1:SYS_NUM,1)=mexstep('get','qvel');
%         y_open(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),j)=Ck*x2-Y_NORM(:,i+1);
%     end
%     % closedloop
%     mexstep('reset');
%     mexstep('set','qpos',X_INIT_TEST(1:POS_NUM,1),POS_NUM);
%     mexstep('set','qvel',X_INIT_TEST(POS_NUM+1:SYS_NUM,1),VEL_NUM);
%     mexstep('forward');
%     for i=1:1:STEP_NUM
%         x1(1:POS_NUM,1)=mexstep('get','qpos');
%         x1(POS_NUM+1:SYS_NUM,1)=mexstep('get','qvel');
%         y_closed(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+2),j)=Ck*x1-Y_NORM(:,i);
%         if i >= max(q,qu)+2
% %             u_feedback=-TK(:,:,i)*[y_closed(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),j);delta_u_test(IN_NUM*(STEP_NUM-i+2)+1:IN_NUM*(STEP_NUM-i+2+qu),j)];
%             u_feedback(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j)=-TK(:,:,i)*[y_closed(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),j);u_feedback(IN_NUM*(STEP_NUM-i+2)+1:IN_NUM*(STEP_NUM-i+2+qu),j)];
% 
%             mexstep('set','ctrl',u_norm(:,i)+delta_u_test(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j)+u_feedback(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j),IN_NUM);
%         else
%             mexstep('set','ctrl',u_norm(:,i),IN_NUM);
%         end
%         mexstep('step',1);
%     end
%     x1(1:POS_NUM,1)=mexstep('get','qpos');
%     x1(POS_NUM+1:SYS_NUM,1)=mexstep('get','qvel');
%     y_closed(1:OUT_NUM,j)=Ck*x1-Y_NORM(:,STEP_NUM+1);
% %     % openloop
% %     x1 = X_INIT_TEST(:,1);
% %     for i=max(q,qu)+1:1:STEP_NUM_SHOW
% %         x2=Ak(:,:,i)*x1;%+Bk*(u_norm(:,i)+delta_u_test(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j));
% %         y_open(OUT_NUM*(STEP_NUM_SHOW-i+1)+1:OUT_NUM*(STEP_NUM_SHOW-i+2),j)=Ck*x1;%+Dk*(u_norm(:,i)+delta_u_test(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j))-Y_NORM(:,i);
% %         x1=x2;
% %     end
% %     y_open(1:OUT_NUM,j)=Ck*x1;%-Y_NORM(:,STEP_NUM+1);
% %     x1 = X_INIT_TEST(:,1);u_feedback=0;
% %     % closedloop correct when Dk=0
% %     for i=max(q,qu)+1:1:STEP_NUM_SHOW
% %         y_closed(OUT_NUM*(STEP_NUM_SHOW-i+1)+1:OUT_NUM*(STEP_NUM_SHOW-i+2),j)=Ck*x1;%+Dk*(u_norm(:,i)+delta_u_test(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j))-Y_NORM(:,i);
% %         u_feedback=-TK(:,:,i)*[y_closed(OUT_NUM*(STEP_NUM_SHOW-i+1)+1:OUT_NUM*(STEP_NUM_SHOW-i+1+q),j);delta_u_test(IN_NUM*(STEP_NUM_SHOW-i+2)+1:IN_NUM*(STEP_NUM_SHOW-i+2+qu),j)];
% %         x2=Ak(:,:,i)*x1+Bk(:,:,i)*(u_feedback);%u_norm(:,i)+delta_u_test(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j)+
% %         x1=x2;
% %     end
% %     y_closed(1:OUT_NUM,j)=Ck*x1;%-Y_NORM(:,STEP_NUM+1);
% end
% 
% %% results
% % controllability and observability
% % controllability=min(ctmr(max(q,qu)+1:end))
% % observability=rank([Ck;Ck*Ak(:,:,6)])
% % observability=min(obmr(max(q,qu)+1:end))
% 
% % lqr response figure
% y_closed_avg=mean((y_closed),2);
% y_open_avg=mean((y_open),2);
% ropenavg=reshape(y_open_avg,OUT_NUM,STEP_NUM_SHOW+1);
% rclosedavg=reshape(y_closed_avg,OUT_NUM,STEP_NUM_SHOW+1);
% figure()
% % plot([fliplr(ropenavg(1,:))' fliplr(rclosedavg(1,:))' fliplr(ropenavg(2,:))' fliplr(rclosedavg(2,:))']);
% plot([fliplr(ropenavg(1,1:STEP_NUM_SHOW-max(q,qu)))' fliplr(rclosedavg(1,1:STEP_NUM_SHOW-max(q,qu)))']);
% % plot(fliplr(rclosedavg(1,1:STEP_NUM_SHOW-max(q,qu)))');
% % legend('openloop1','closedloop1','openloop2','closedloop2');
% legend('openloop1','closedloop1');
% % openloop=mean(y_open_avg)
% closedloop=mean(abs(y_closed_avg))
% % figure()
% % plot(yy(1,:))
% toc;