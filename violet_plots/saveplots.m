%% Save the figures as PDFs, same way as shuttle_sweep_plot
% results/<bag name>_<timestamp>/<bag name>_{xyz,xy,error,attitude_error}.pdf
% The results folder sits next to the bags folder.
[bagsRoot, bagStem, bagExt] = fileparts(bagFolder);
bagName = [bagStem bagExt];            % keeps decimals such as kphi0.1 in the name
runName = [bagName '_' char(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'))];
resultFolder = fullfile(fileparts(bagsRoot), 'results', runName);
mkdir(resultFolder);

% The vehicle mesh is heavy, so the two figures with it are saved as 600 dpi
% images inside the PDF; the line plots stay vector.
figs = {fig1, 'xyz', 'image'; fig2, 'xy', 'image'; fig3, 'error', 'vector'};
if exist('fig4','var')
    figs(end+1,:) = {fig4, 'attitude_error', 'vector'};
end
for kk = 1:size(figs,1)
    % Fixed page size (no tight cropping), so all PDFs are identical in size.
    set(figs{kk,1}, 'PaperUnits', 'centimeters', 'PaperSize', figureSize, ...
        'PaperPosition', [0 0 figureSize], 'InvertHardcopy', 'off');
    pdfFile = fullfile(resultFolder, [bagName '_' figs{kk,2} '.pdf']);
    print(figs{kk,1}, pdfFile, '-dpdf', ['-' figs{kk,3}], '-r600');
    fprintf('Saved plot: %s\n', pdfFile);
end

matFile = fullfile(resultFolder, [bagName '_data.mat']);
save(matFile, 't', 'pos', 'pd', 'v', 'vd', 'gamma_dot', 'R', 'bagFolder');
if exist('eR','var'), save(matFile, 'eR', '-append'); end
fprintf('Saved MATLAB data: %s\n', matFile);