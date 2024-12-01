function compare_handover_methods(time_history, ...
    ping_original, leo_original, ...
    ping_kalman, leo_kalman, ...
    ping_hmm, leo_hmm, ...
    ping_keplerian, leo_keplerian, ...
    dt)

    % Create comparison metrics
    avg_ping_original = mean(ping_original(ping_original < inf));
    avg_ping_kalman = mean(ping_kalman(ping_kalman < inf));
    avg_ping_hmm = mean(ping_hmm(ping_hmm < inf));
    avg_ping_keplerian = mean(ping_keplerian(ping_keplerian < inf));    
    
    handovers_original = sum(diff(leo_original) ~= 0);
    handovers_kalman = sum(diff(leo_kalman) ~= 0);
    handovers_hmm = sum(diff(leo_hmm) ~= 0);
    handovers_keplerian = sum(diff(leo_keplerian) ~= 0);
    
    coverage_gaps_original = sum(ping_original == inf);
    coverage_gaps_kalman = sum(ping_kalman == inf);
    coverage_gaps_hmm = sum(ping_hmm == inf);
    coverage_gaps_keplerian = sum(ping_keplerian == inf);
    % Plot comparison
    figure('Position', [100, 100, 1200, 800]);
    
    % Ping comparison subplot
    subplot(2,1,1);
    plot(time_history, ping_original, 'b-', 'DisplayName', 'Original Method');
    hold on;
    plot(time_history, ping_kalman, 'r--', 'DisplayName', 'Kalman Filter');
    plot(time_history, ping_hmm, 'g--', 'DisplayName', 'HMM');
    plot(time_history, ping_keplerian, 'm--', 'DisplayName', 'Keplerian');
    grid on;
    xlabel('Time (s)');
    ylabel('Ping (ms)');
    title('Ping Comparison');
    legend('show');
    
    % LEO connection subplot
    subplot(2,1,2);
    plot(time_history, leo_original, 'b-', 'DisplayName', 'Original Method');
    hold on;
    plot(time_history, leo_kalman, 'r--', 'DisplayName', 'Kalman Filter');
    plot(time_history, leo_hmm, 'g--', 'DisplayName', 'HMM');
    plot(time_history, leo_keplerian, 'm--', 'DisplayName', 'Keplerian');
    grid on;
    xlabel('Time (s)');
    ylabel('Connected LEO');
    title('LEO Connection Comparison');
    legend('show');
    
    % Display metrics
    fprintf('\nComparison Metrics:\n');
    fprintf('Original Method:\n');
    fprintf('  Average Ping: %.2f ms\n', avg_ping_original);
    fprintf('  Number of Handovers: %d\n', handovers_original);
    fprintf('  Coverage Gaps: %d\n', coverage_gaps_original);
    
    fprintf('\nKalman Filter Method:\n');
    fprintf('  Average Ping: %.2f ms\n', avg_ping_kalman);
    fprintf('  Number of Handovers: %d\n', handovers_kalman);
    fprintf('  Coverage Gaps: %d\n', coverage_gaps_kalman);
    
    fprintf('\nHMM Method:\n');
    fprintf('  Average Ping: %.2f ms\n', avg_ping_hmm);
    fprintf('  Number of Handovers: %d\n', handovers_hmm);
    fprintf('  Coverage Gaps: %d\n', coverage_gaps_hmm);

    fprintf('\nKeplerian Method:\n');
    fprintf('  Average Ping: %.2f ms\n', avg_ping_keplerian);
    fprintf('  Number of Handovers: %d\n', handovers_keplerian);
    fprintf('  Coverage Gaps: %d\n', coverage_gaps_keplerian);
    
    % Save comparison data
    comparison_data = table(time_history, ping_original, leo_original, ...
                          ping_kalman, leo_kalman, ...
                          ping_hmm, leo_hmm, ...
                          ping_keplerian, leo_keplerian, ...
                          'VariableNames', {'Time', 'Ping_Original', 'LEO_Original', ...
                                          'Ping_Kalman', 'LEO_Kalman', ...
                                          'Ping_HMM', 'LEO_HMM', ...
                                          'Ping_Keplerian', 'LEO_Keplerian'});
    writetable(comparison_data, 'handover_comparison.csv');
end