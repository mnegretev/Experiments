%Read data from files
DataJustina = csvread('results_nagoya_justina2.mpd',1,0);
DataNav2d   = csvread('results_nagoya_nav2d.mpd', 1, 0);

%Calculate statistics
justina_time_mean  = mean(DataJustina(:,3));
justina_time_std   = std(DataJustina(:,3));
justina_dist_mean  = mean(DataJustina(:,4));
justina_dist_std   = std(DataJustina(:,4));
justina_theta_mean = mean(DataJustina(:,5));
justina_theta_std  = std(DataJustina(:,5));
justina_crash_mean = mean(DataJustina(:,6));
justina_crash_std  = std(DataJustina(:,6));
justina_crash_sum  = sum(DataJustina(:,6));

nav2d_time_mean  = mean(DataNav2d(:,3));
nav2d_time_std   = std(DataNav2d(:,3));
nav2d_dist_mean  = mean(DataNav2d(:,4));
nav2d_dist_std   = std(DataNav2d(:,4));
nav2d_theta_mean = mean(DataNav2d(:,5));
nav2d_theta_std  = std(DataNav2d(:,5));
nav2d_crash_mean = mean(DataNav2d(:,6));
nav2d_crash_std  = std(DataNav2d(:,6));
nav2d_crash_sum  = sum(DataNav2d(:,6));