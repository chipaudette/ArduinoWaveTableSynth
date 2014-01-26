
fname = 'MonoPoly_Saw_FilterSweepRes0.wav';prefix = 'saw_Res0_';
%fname = 'MonoPoly_Saw_FilterSweepRes4.wav';prefix = 'saw_Res4_';
%fname = 'MonoPoly_Saw_FilterSweepRes7.wav';prefix = 'saw_Res7_';
%fname = 'MonoPoly_Sqr_FilterSweepRes0.wav';prefix = 'sqr_Res0_';
%fname = 'MonoPoly_Saw_FilterSweepRes4.wav';prefix = 'sqr_Res4_';

disp(['loading ' fname]);
[wav,fs]=wavread(fname);

%high pass filter to remove DC shift
[b,a]=butter(2,1/(fs/2),'high');
wav = filter(b,a,wav);

%remove first half second
wav = wav(fs/2:end);

outpname='01-CutUp_Raw\';


%% find all single waves while signal is large enough
% thresh = 0.021;Iend=find(wav > thresh);Iend=Iend(end);
% ind = find((wav(1:Iend-1) < 0) & (wav(2:Iend) >= 0)); %find upward zero crossings
% ind = [ind(1:end-1) ind(2:end)];
% diff_ind = diff(ind')';
% med_diff_ind = median(diff_ind);
% I=find(abs(diff_ind-med_diff_ind) < 0.1*med_diff_ind);
% ind = ind(I,:);

thresh = 0.01;black_out = (5e-3*fs);
ind_up = [];ind_down=[];looking_for_up=1;
for I=1:length(wav);
    %if (rem(I,1000)==0);disp(['computing ' num2str(I) ' of ' num2str(length(wav))]);end
    switch looking_for_up
        case -1.0
            %looking for down
            if (wav(I) < -thresh)
                ind_down(end+1)=I;
                looking_for_up = 0.5;  %blackout
            end
        case -0.5
            %black-out
            if (I-ind_up(end)) > black_out
                looking_for_up = -1;  %look for up
            end
        case +0.5
            %black-out
            if (I-ind_down(end)) > black_out
                looking_for_up = 1;  %look for down
            end
        case +1.0
            %look for up
            if (wav(I) > thresh)   
                ind_up(end+1)=I;
                looking_for_up = -0.5;
            end
    end
end
ind = ind_up(2:end);

%back up to zero crossing
for Iind = 1:length(ind);
    if Iind==1
        I=find(wav(1:ind(1)) < 0);
        if isempty(I);I=0;end;
        ind(Iind)=I(end)+1;
    else
        I=find(wav(ind(Iind-1):ind(Iind)) < 0);
        ind(Iind)=I(end)+ind(Iind-1);
    end
end
        
%define start and end 
ind = [ind(1:end-1)' ind(2:end)']

% regularize their length
new_diff_ind = diff(ind')';
med_diff_ind = 2*round(0.5*median(new_diff_ind));
new_ind = ind(:,1);
new_ind(:,2) = ind(:,1)+med_diff_ind-1;

t_sec = ([1:length(wav)]-1)/fs;
t_block_sec = mean(t_sec(new_ind)')';

%calc cutoff
cutoff_bin=NaN*ones(size(new_ind,1),1);
foo_wav = wav(new_ind(1,1):new_ind(1,2));
N = length(foo_wav);
baseline_pow = fft(foo_wav).*conj(fft(foo_wav))./N^2;
baseline_pow = baseline_pow(1:N/2+1);
freq_Hz = [0:1/N:1]*fs;freq_Hz = freq_Hz(1:N/2+1);
norm_pow = max(baseline_pow);
baseline_pow = baseline_pow/norm_pow;
thresh = 10.^(0.1*-40);
for Iind = 1:size(new_ind,1)
    foo_wav = wav(new_ind(Iind,1):new_ind(Iind,2));
    pow = fft(foo_wav).*conj(fft(foo_wav))./N^2;
    pow = pow(1:N/2+1);
    pow = pow/norm_pow;
    
    rel_pow = pow./baseline_pow;
    I=find((baseline_pow > thresh) & (rel_pow > 0.25));
    if ~isempty(I);
        cutoff_bin(Iind)=I(end);
    else
        cutoff_bin(Iind)=length(rel_pow);
    end
    
%     if Iind==1;figure;setFigureTall;end;
%     subplot(2,1,1);
%     plot(freq_Hz,10*log10(baseline_pow),freq_Hz,10*log10(pow));
%     ylim([-40 0]);
%     title(['Iind = ' num2str(Iind) ' of ' num2str(size(new_ind,1))]);
%     xlim([0 6000]);
%     
%     subplot(2,1,2);
%     plot(freq_Hz,10*log10(rel_pow));
%     ylim([-20 5]);
%     xlim([0 6000]);
%     xlabel('Freq (Hz)');
%     ylabel('Rel Pow (dB/bin)');
%     hold on;
%     plot(freq_Hz(cutoff_bin(Iind)),10*log10(rel_pow(cutoff_bin(Iind))),'ro','linewidth',2);
%     hold off
%     drawnow;
%     pause(0.25);
end
block_freq_Hz = freq_Hz(cutoff_bin);
% dfreq = diff(block_freq_Hz);
% I=find(dfreq > 50);
% if ~isempty(I)
%     block_freq_Hz(I(1):end) = block_freq_Hz(I(1)-1);
%     cutoff_bin(I(1):end) = cutoff_bin(I(1)-1);
% end

%pick the best blocks based on exponential freq spacing

I = find(~isnan(block_freq_Hz) & (block_freq_Hz > 20));
min_freq = min(block_freq_Hz(I));
max_freq = min([10000 max(block_freq_Hz(I))]);
n_blocks_des = 5;
targ_freq_Hz = exp(interp1([0 1],log([max_freq min_freq]),[0:1/(n_blocks_des-1):1]));
targ_block_ind=[];
for Itarg = 1:length(targ_freq_Hz);
    [foo,I]=min(abs(block_freq_Hz - targ_freq_Hz(Itarg)));
    targ_block_ind(Itarg) = I;
end


%write the waves to files
out_wav = wav - mean(wav);
out_wav = out_wav./max(abs(out_wav));
figure;setFigureTallWide;
for Iout = 1:length(targ_block_ind);
    out_ind = ind(targ_block_ind(Iout),:);
    outfname=[prefix '0' num2str(Iout) '.wav'];
    disp(['writing wav to ' outpname outfname]);
    subplot(4,2,Iout);
    foo_out = out_wav(out_ind(1):out_ind(2));
    plot(foo_out);title(num2str(Iout));
    wavwrite(foo_out,fs,16,[outpname outfname]);
end

    

%% plot
figure;setFigureTall;
subplot(3,1,1);
t_sec = ([1:length(wav)]-1)/fs;
plot(t_sec,wav);
hold on;
yl=ylim;
for I=1:length(ind);
    plot(t_sec(ind(I,1))*[1 1],yl,'g:');
    plot(t_sec(ind(I,2))*[1 1],yl,'r:');
end
xlabel('Time (sec)');
ylabel('Value');
title(fname,'interpreter','none');
xl=xlim;
ax=gca;

subplot(3,1,2);
plot(t_block_sec,(ind(:,2)-ind(:,1)),'go-','linewidth',2);
hold on
plot(t_block_sec,(new_ind(:,2)-new_ind(:,1)),'ro-','linewidth',2);
legend('Zero Cross','Regularized');
%plot(t_sec(ind_down(2:end)),diff(ind_down)/fs,'ro-','linewidth',2);
xlabel('Time (sec)');
ylabel('Sample length');
xlim(xl);
ax(end+1)=gca;

subplot(3,1,3);
semilogy(t_block_sec,freq_Hz(cutoff_bin),'.-');
xlabel('Time (sec)');
ylabel('Cutoff Freq (Hz)');
hold on;
semilogy(t_block_sec(targ_block_ind),block_freq_Hz(targ_block_ind),'mo','linewidth',2);
hold off
ax(end+1)=gca;
ylim([10 11000]);
set(gca,'YTick',[10 100 1000 10000]);
linkaxes(ax,'x');
