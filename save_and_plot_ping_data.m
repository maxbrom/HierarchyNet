function save_and_plot_ping_data(time_history, ping_history, leo_connection_history, dt)
    % Replace inf values with a high but plottable value (200 ms)
    MAX_PING_VALUE = 200;  
    ping_history_plot = ping_history;
    ping_history_plot(isinf(ping_history_plot)) = MAX_PING_VALUE;
    
    % Create a table with the collected data
    data_table = table(time_history, ping_history, leo_connection_history, ...
        'VariableNames', {'Time_Seconds', 'Ping_ms', 'Connected_LEO'});
    
    % Save to CSV
    writetable(data_table, 'ping_history.csv');
    
    % Create figure for the plot
    figure('Position', [100, 100, 1200, 600]);
    
    % Create the main ping vs time plot
    plot(time_history, ping_history_plot, 'b-', 'LineWidth', 1.5);
    hold on;
    
    % Add points only for active LEO connections (when ping is not at max)
    unique_leos = unique(leo_connection_history);
    colors = hsv(length(unique_leos));
    
    for i = 1:length(unique_leos)
        leo = unique_leos(i);
        if leo > 0  % Only plot actual LEO connections (ignore 0/no connection)
            leo_indices = find(leo_connection_history == leo);
            % Only plot points where ping is less than MAX_PING_VALUE
            valid_indices = leo_indices(ping_history_plot(leo_indices) < MAX_PING_VALUE);
            if ~isempty(valid_indices)
                scatter(time_history(valid_indices), ping_history_plot(valid_indices), 50, ...
                    colors(i,:), 'filled', 'DisplayName', ['LEO ' num2str(leo)]);
            end
        end
    end
    
    % Customize the plot
    grid on;
    xlabel('Time (seconds)');
    ylabel('Ping (ms)');
    yscale('log');
    title('Satellite Connection Ping History');
    legend('show', 'Location', 'eastoutside');
    
    % Add time markers on x-axis
    ax = gca;
    ax.XTick = 0:60:max(time_history);  % Markers every 60 seconds
    
    % Format time labels as MM:SS
    time_labels = cell(size(ax.XTick));
    for i = 1:length(ax.XTick)
        minutes = floor(ax.XTick(i)/60);
        seconds = mod(ax.XTick(i), 60);
        time_labels{i} = sprintf('%02d:%02d', minutes, seconds);
    end
    ax.XTickLabel = time_labels;
    
    % Rotate x-axis labels for better readability
    xtickangle(45);
    
    % Add note about maximum ping value
    if any(isinf(ping_history))
        text(0.02, 0.98, sprintf('Note: No connection shown as %d ms', MAX_PING_VALUE), ...
            'Units', 'normalized', 'FontSize', 8, 'Color', 'red');
    end
    
    % Save the plot
    saveas(gcf, 'ping_history_plot.png');
    saveas(gcf, 'ping_history_plot.fig');
end 