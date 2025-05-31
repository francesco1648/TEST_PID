close all;
clear all;
filename = 'file_prova.txt';
fid = fopen(filename, 'r');
if fid == -1
    error('Impossibile aprire il file.');
end

% Campi da estrarre
campi = {'tempo', 'getSpeed', 'feedback', 'output', 'fe', 'derivative', ...
         'integral', 'old_integral', 'old_error', 'old_fe', ...
         'analogWrite', 'digitalWrite', 'getOutput', 'speedtoPower'};

% Inizializza le strutture
leftData = initStruttura(campi);
rightData = initStruttura(campi);

while ~feof(fid)
    line = strtrim(fgetl(fid));

    if contains(line, 'getSpeed')
        % Estrazione
        dati = estraiDati(line, campi);

        if contains(line, 'MOTOR_LEFT') && ~contains(line, 'MOTOR_RIGHT getSpeed')
            % Solo MOTOR_LEFT ha dati
            leftData = aggiungi(leftData, dati, campi);
        elseif contains(line, 'MOTOR_RIGHT') && contains(line, 'MOTOR_LEFT')
            % MOTOR_RIGHT ha i dati
            rightData = aggiungi(rightData, dati, campi);
        end
    end
end

fclose(fid);

% Plot per ciascun campo
plotDati(leftData, 'MOTOR_LEFT');
plotDati(rightData, 'MOTOR_RIGHT');

% ---------- Funzioni ------------

function struttura = initStruttura(campi)
    struttura = struct();
    for i = 1:length(campi)
        struttura.(campi{i}) = [];
    end
end

function struttura = aggiungi(struttura, dati, campi)
    for i = 1:length(campi)
        campo = campi{i};
        struttura.(campo)(end+1) = dati.(campo);
    end
end

function dati = estraiDati(riga, campi)
    dati = struct();
    for i = 1:length(campi)
        campo = campi{i};
        pattern = [campo '\s+([-+]?[0-9]*\.?[0-9]+)'];
        token = regexp(riga, pattern, 'tokens', 'once');
        if ~isempty(token)
            dati.(campo) = str2double(token{1});
        else
            dati.(campo) = NaN;
        end
    end
end

function plotDati(data, motore)
    tempo = data.tempo;
    campi = fieldnames(data);
    for i = 1:length(campi)
        campo = campi{i};
        if strcmp(campo, 'tempo'), continue; end
        valori = data.(campo);

        % Verifica se ci sono dati validi
        if all(isnan(valori)), continue; end

        figure;
        plot(tempo, valori, '-o');
        title([motore ' - ' campo]);
        xlabel('tempo (ms)');
        ylabel(campo);
        grid on;
    end
end
