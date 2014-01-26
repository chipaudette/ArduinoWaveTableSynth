
pname = 'CutUpVowels_Processed\';
fnames={};

switch 2
    case 1
        fnames = {'A1' 'A2' 'A3' 'A8' 'E1' 'E2' 'E5' 'E8' 'I2' 'I4' 'I6' 'I8' ...
            'O2' 'O4' 'O6' 'O8' 'U1' 'U3' 'U4' 'U5'};
        %fnames = {'A3.wav';'E5.wav';'I6.wav';'O4.wav';'U4.wav'};
        %fnames = {'E1.wav' 'E2.wav' 'E5.wav' 'E8.wav'};
        %fnames = {'I2.wav';'I4.wav';'I6.wav';'I8.wav'};
        %fnames = {'O2.wav';'O4.wav';'O6.wav';'O8.wav'};
        %fnames = {'U1.wav' 'U3.wav' 'U4.wav' 'U5.wav'};
        disp(['#define N_POINTS_WAVE (64)']);
    case 2
        wave_len = 256*4;  %128 or 256 or whatever
        outfname = ['vowels_' num2str(wave_len) '.h'];
        disp(['saving to ' outfname]);
        eval(['!del ' outfname]);
        diary(outfname);
        for I=1:25;
            if I<10
                fnames{I} = ['0' num2str(I) '_' num2str(wave_len) '.wav'];
            else
                fnames{I} = [num2str(I) '_' num2str(wave_len) '.wav'];
            end
        end
        disp(['#define N_POINTS_WAVE (' num2str(wave_len) ')']);
end

disp(['#define N_WAVES (' num2str(length(fnames)) ')']);
disp(['prog_int16_t waveTable[N_WAVES][N_POINTS_WAVE] PROGMEM  = {']);

nbits_down = 2; %make full scale 14 bis
div_fac = 2^nbits_down;
%targ_len = 64;
for Ifile=1:length(fnames);
    fname = fnames{Ifile};
    [wav,fs] = wavread([pname fname]);
    wav = round((wav * 2^(15))/div_fac);
      
    npts = length(wav);
    endloop = ceil(npts/4);
    for I=1:endloop;
        inds = (I-1)*4+[1:4];
        fmt = '%i, %i, %i, %i';
        if I==1;fmt=['{' fmt];end;
        if I==endloop;
            fmt=[fmt '}'];
            if Ifile<length(fnames)
                fmt=[fmt ','];
            end
        else;
            fmt=[fmt ','];
        end;
        disp(sprintf(fmt,wav(inds)));
    end
    disp(' ');
end
disp(['};']);
diary('off');
    