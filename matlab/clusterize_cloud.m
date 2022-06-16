function [clusters_xyz, clusters_rgb, centroids] = clusterize_cloud(xyz, rgb)
    K = 15;
    [idx, centroids] = kmeans(xyz, K);
    d = min(pdist(centroids));
    while d < 0.09 && K > 0
        K = K - 1;
        [idx, centroids] = kmeans(xyz, K);
        if K > 1
            d = min(pdist(centroids));
        else
            d = inf;
        end
    end
    clusters_xyz = {};
    clusters_rgb = {};
    counters = ones(length(idx), 1);
    for i=1:length(idx)
        clusters_xyz{idx(i)}(counters(idx(i)),:) = xyz(i,:);
        clusters_rgb{idx(i)}(counters(idx(i)),:) = rgb(i,:);
        counters(idx(i)) = counters(idx(i)) + 1;
    end
    