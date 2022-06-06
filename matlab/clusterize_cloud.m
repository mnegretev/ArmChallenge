function [clusters_xyz, clusters_rgb, centroids] = clusterize_cloud(xyz, rgb)
    K = 15;
    [idx, centroids] = kmeans(xyz, K);
    d = min(pdist(centroids));
    while d < 0.11 && K > 0
        K = K - 1;
        [idx, centroids] = kmeans(xyz, K);
        d = min(pdist(centroids));
    end
    clusters_xyz = {};
    clusters_rgb = {};
    counters = ones(length(idx), 1);
    for i=1:length(idx)
        clusters_xyz{idx(i)}(counters(idx(i)),:) = xyz(i,:);
        clusters_rgb{idx(i)}(counters(idx(i)),:) = rgb(i,:);
        counters(idx(i)) = counters(idx(i)) + 1;
    end
    %figure
    %pcshow(pointCloud(xyz, 'Color', uint8(rgb*255)));
    %hold on
    %scatter3(centroids(:,1), centroids(:,2), centroids(:,3), 200, "red", 'filled', 'o');
    %hold off