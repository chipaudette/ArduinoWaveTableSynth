
fnames={};

resample_wavs=0;
switch 99
    case 1
        pname = '02-Vowels\02-CutUpVowels_Processed\';
        wave_len = 256*4;  %128 or 256 or whatever
        outfname = ['vowels_' num2str(wave_len) '.h'];
        disp(['saving to ' outfname]);
        eval(['!del ' outfname]);
        diary(outfname);
        fnames=[];
        for I=1:25;
            if I<10
                fnames{I} = [pname '0' num2str(I) '_' num2str(wave_len) '.wav'];
            else
                fnames{I} = [pname num2str(I) '_' num2str(wave_len) '.wav'];
            end
        end
    case 2
        pname = '03-MonoPoly\01-CutUp_Raw\';
        wave_len = 256*4;  %128 or 256 or whatever
        outfname = ['monoPoly_' num2str(wave_len) '.h'];
        prefixes = {'saw_Res0_0';'saw_Res4_0';'sqr_Res0_0';'sqr_Res4_0'};
        fnames={};
        for Iprefix=1:length(prefixes)
            for I=1:5;
                fnames{end+1}=[pname prefixes{Iprefix} num2str(I) '.wav'];
            end
        end
        resample_wavs=1;
        disp(['saving to ' outfname]);
        eval(['!del ' outfname]);
        diary(outfname);
    case 99
        %lots together
        wave_len = 256*2;
        outfname = ['combined_' num2str(wave_len) '.h'];
        prefixes = {'saw_Res0_0';'saw_Res4_0';'sqr_Res0_0';'sqr_Res4_0'};
        fnames={};
        %filenames for monopoly waves
        pname = '03-MonoPoly\01-CutUp_Raw\';
        for Iprefix=1:length(prefixes)
            for I=1:5;
                fnames{end+1}=[pname prefixes{Iprefix} num2str(I) '.wav'];
            end
        end
        %filenames for vowels
        pname = '02-Vowels\02-CutUpVowels_Processed\';
        for I=1:25;
            if I<10
                fnames{end+1} = [pname '0' num2str(I) '_1024.wav'];
            else
                fnames{end+1} = [pname num2str(I) '_1024.wav'];
            end
        end
        resample_wavs=1;
        disp(['saving to ' outfname]);
        eval(['!del ' outfname]);
        diary(outfname);
end

disp(['#define N_POINTS_WAVE (' num2str(wave_len) ')']);
disp(['#define N_BITS_LEN_WAVES (' num2str(log(wave_len)/log(2)) ')']);
disp(['#define N_WAVES (' num2str(length(fnames)) ')']);
disp(['#define N_BITS_WAVE_RES (14)']);
disp(['//prog_int16_t waveTable[N_WAVES][N_POINTS_WAVE] PROGMEM  = {']);  % for arduino
disp(['int16 waveTable[N_WAVES][N_POINTS_WAVE] __FLASH__ =  {']);  % for Maple

nbits_down = 2; %make full scale 14 bis
div_fac = 2^nbits_down;
%targ_len = 64;
for Ifile=1:length(fnames);
    fname = fnames{Ifile};
    [wav,fs] = wavread(fname);
    if (resample_wavs)
        if ~(length(wav) == wave_len)
            wav = resample(wav,wave_len,length(wav));
        end
    end
    wav = round((wav * 2^(15))/div_fac);

    npts = length(wav);
    endloop = ceil(npts/8);
    for I=1:endloop;
        inds = (I-1)*8+[1:8];
        fmt = '%i, %i, %i, %i, %i, %i, %i, %i';
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
