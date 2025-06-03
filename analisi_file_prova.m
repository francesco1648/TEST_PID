close all;
clear all;

% === LETTURA FILE ===
filename = 'static_pid2.txt';
fid = fopen(filename, 'r');
if fid == -1
    error('Impossibile aprire il file.');
end

% Campi da estrarre
campi = {'tempo', 'getReferenceValue', 'getSpeed', 'feedback', 'output', 'fe', 'derivative', ...
         'integral', 'old_integral', 'old_error', 'old_fe', ...
         'analogWrite', 'digitalWrite', 'getOutput', 'speedtoPower'};

% Inizializza le strutture
leftData = initStruttura(campi);
rightData = initStruttura(campi);

while ~feof(fid)
    line = strtrim(fgetl(fid));

    if contains(line, 'getSpeed')
        dati = estraiDati(line, campi);

        if contains(line, 'MOTOR_LEFT') && ~contains(line, 'MOTOR_RIGHT getSpeed')
            leftData = aggiungi(leftData, dati, campi);
        elseif contains(line, 'MOTOR_RIGHT') && contains(line, 'MOTOR_LEFT')
            rightData = aggiungi(rightData, dati, campi);
        end
    end
end

fclose(fid);

% === PLOT ===
plotCampiSeparati(leftData, rightData);
plotReferenceSpeedAnalog(leftData, 'MOTOR_LEFT');
plotReferenceSpeedAnalog(rightData, 'MOTOR_RIGHT');

% ---------- FUNZIONI ------------

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

function plotCampiSeparati(leftData, rightData)
    campi = fieldnames(leftData);
    campi(strcmp(campi, 'tempo')) = [];  % Rimuove 'tempo'

    for i = 1:length(campi)
        campo = campi{i};

        % --- MOTOR_LEFT ---
        tempoLeft = leftData.tempo;
        valoriLeft = leftData.(campo);
        validiLeft = ~isnan(valoriLeft);

        if any(validiLeft)
            figure('Name', ['MOTOR_LEFT - ' campo], 'NumberTitle', 'off');
            plot(tempoLeft(validiLeft), valoriLeft(validiLeft), 'b');
            title(['MOTOR\_LEFT - ' campo], 'Interpreter', 'none');
            xlabel('tempo (ms)');
            ylabel(campo);
            grid on;
        end

        % --- MOTOR_RIGHT ---
        tempoRight = rightData.tempo;
        valoriRight = rightData.(campo);
        validiRight = ~isnan(valoriRight);

        if any(validiRight)
            figure('Name', ['MOTOR_RIGHT - ' campo], 'NumberTitle', 'off');
            plot(tempoRight(validiRight), valoriRight(validiRight), 'r');
            title(['MOTOR\_RIGHT - ' campo], 'Interpreter', 'none');
            xlabel('tempo (ms)');
            ylabel(campo);
            grid on;
        end
    end
end

function plotReferenceSpeedAnalog(data, motore)
    tempo = data.tempo;
    
    ref = data.getReferenceValue;
    speed = data.getSpeed;
    pwm = data.analogWrite;
    mg=data.digitalWrite;

    validi = ~isnan(ref) & ~isnan(speed) & ~isnan(pwm);

    if any(validi)
        figure('Name', [motore ' - reference vs speed vs analogWrite'], 'NumberTitle', 'off');
        plot(tempo(validi), ref(validi), 'k-', 'LineWidth', 1.5, 'DisplayName', 'getReferenceValue');
        hold on;
        plot(tempo(validi), speed(validi), 'b--', 'LineWidth', 1.5, 'DisplayName', 'getSpeed');
        plot(tempo(validi), pwm(validi), 'r:', 'LineWidth', 1.5, 'DisplayName', 'analogWrite');
       plot(tempo(validi), mg(validi)*60,  'LineWidth', 1.5, 'DisplayName', 'digitalWrite');
       
        title([motore ' - Reference vs Speed vs analogWrite'], 'Interpreter', 'none');
        xlabel('tempo (ms)');
        ylabel('Valori');
        legend;
        grid on;
        hold off;
    end
end
