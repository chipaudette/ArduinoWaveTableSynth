pname = 'CutUpVowels_Raw\';
outpname_wav = 'CutUpVowels_Processed\';
fnames = dir([pname '*.wav']);

nbits_down = 2; %make full scale 14 bis
div_fac = 2^nbits_down;
targ_len = 256*4;
for Ifile=1:length(fnames);
    disp(['processing ' num2str(Ifile) ' of ' num2str(length(fnames))]);
    fname = fnames(Ifile).name;
    [wav,fs] = wavread([pname fname]);
    wav = wav / max(abs(wav));
    wav = resample(wav,targ_len,length(wav));
    
    %fade in
    nfade = round(6*targ_len/256);
    gain = interp1([1 nfade],[0 1],[1:nfade]);
    wav(1:nfade)=gain(:).*wav(1:nfade);
    
    %fade out
    wav = wav(end:-1:1);
    wav(1:nfade)=gain(:).*wav(1:nfade);
    wav = wav(end:-1:1);
    
    %write processed wav
    wavwrite(wav,110*64,16,[outpname_wav fname(1:end-4) '_' num2str(targ_len) '.wav']);
end
    
    
%     
%     npts = length(wav);
%     %disp('{');
%     wav = round(wav/div_fac);
%     endloop = ceil(npts/4);
%     for I=1:endloop;
%         inds = (I-1)*4+[1:4];
%         fmt = '%i, %i, %i, %i';
%         if I==1;fmt=['{' fmt];end;
%         if I==endloop;
%             fmt=[fmt '}'];
%             if Ifile<length(fnames)
%                 fmt=[fmt ','];
%             end
%         else;
%             fmt=[fmt ','];
%         end;
%         disp(sprintf(fmt,wav(inds)));
%     end
%     disp(' ');
% end

    