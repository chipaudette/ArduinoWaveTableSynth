fnames = {'Saw_64pt.txt';
    'SawFormant_64pt.txt';
    'Square_64pt.txt';
    'EnvNoise_64pt.txt';
    'Sine2 _64pt.txt'};

disp(['#define N_POINTS_WAVE (64)']);
disp(['#define N_WAVES (' num2str(length(fnames)) ')']);
disp(['int waveTable[N_WAVES][N_POINTS_WAVE] = {']);

nbits_down = 2; %make full scale 14 bis
div_fac = 2^nbits_down;
for Ifile=1:length(fnames);
    wav = textread(fnames{Ifile},'%f','headerlines',5);
    npts = length(wav);
    %disp('{');
    wav = round(wav/div_fac);
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

